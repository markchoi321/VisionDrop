//
//  DenseDecoder.swift
//  VisionDrop
//
//  VD-D1 密集彩色 L1 的 Swift 封装。重活全部在 C 核心（vd_dense_core.c）
//  里完成：亮度游程定位、单应性、逐格采样、定标判决、RS errata 纠错。
//  这里只做 CVPixelBuffer / Data 到 C API 的零拷贝桥接与跟踪状态管理。
//
//  相机路径约定：AVCaptureVideoDataOutput 配成 kCVPixelFormatType_32BGRA，
//  YUV->BGRA 由 ISP 硬件完成，C 核心直接读 BGRA。
//
//  解出的帧字节交给现有 FrameParser，L2 之上不感知物理层差异。
//

import CoreVideo
import Foundation

final class DenseDecoder {

    /// 解码失败原因，与 C 返回码一一对应
    enum Failure: Int32, Error {
        case badArgs = -1
        case finderNotFound = -2
        case headerInvalid = -3
        case rsUncorrectable = -4
        case capacityMismatch = -5
        case outOfMemory = -6
        case calibrationFailed = -7
        /// 信噪比过低（低置信格超过四分之一），整帧放弃，不值得重试
        case signalTooWeak = -8

        /// 遥测日志用的短名。日志里写裸数字（-4）事后没人看得懂，
        /// 而界面文案又太长且会变，因此单列一套稳定的机器可读名字。
        var logName: String {
            switch self {
            case .badArgs: return "bad_args"
            case .finderNotFound: return "finder"
            case .headerInvalid: return "header"
            case .rsUncorrectable: return "rs"
            case .capacityMismatch: return "capacity"
            case .outOfMemory: return "nomem"
            case .calibrationFailed: return "calib"
            case .signalTooWeak: return "weak"
            }
        }
    }

    /// 自动判别档位。符号不携带 N，手工对齐两端档位极易出错——
    /// 真机首测就栽在这里（发送端 128 格、接收端默认 96 格，
    /// 定位图形照样找得到，但格心全偏，表现为「头部校验失败」）。
    static let gridAuto: Int32 = 0

    /// 最近一次解码的测量值，用于界面诊断与真机标定
    private(set) var diagnostics = vdd_diag()

    /// 跨帧跟踪状态：锁定后只在四角小窗内精定位，丢锁自动回退全图搜索
    private var lock = vdd_lock()
    private var frameBuf = [UInt8](repeating: 0, count: 8192)

    /// 上一个成功帧的字节。C 核心判出「与上一成功帧同一符号」（VDD_DUPLICATE）
    /// 时不做全采样与纠错，也不写输出缓冲，这里原样交还上一帧的字节。
    private var lastFrame: Data?

    /// 最近一次 decode 返回的字节是否来自重复帧早退（未做全采样与纠错）。
    /// 返回值本身与完整解码完全相同，调用方不处理也不会出错；遥测想区分
    /// 「C 层判重」与「解出后按哈希去重」时读它。
    private(set) var lastWasDuplicate = false

    /// 当前是否处于跨帧跟踪锁定态（只在四角小窗内精定位）。
    /// 遥测日志要区分「锁定态下的失败」与「全图搜索下的失败」——前者多半是
    /// 信噪比问题，后者更可能是取景问题。
    var isTracking: Bool { lock.valid != 0 }

    /// 该 N 档的净载荷上限（字节），非法 N 返回 0。colors 取 8（缺省）或 4（降级档）
    static func netCapacity(grid n: Int32, colors: Int32 = 8) -> Int32 {
        vdd_net_capacity_colors(n, colors)
    }

    /// RS errata 随机回归（调试用）
    static func rsSelfTest(trials: Int32 = 500) -> Bool {
        vdd_selftest_rs(trials) == 0
    }

    /// 重置跟踪状态（会话切换或长时间丢锁时调用）
    func resetLock() {
        lock.valid = 0
        lock.grid = 0
        lock.beta = 0
        lock.dup_n = 0
        lastFrame = nil
    }

    /// 从 BGRA CVPixelBuffer 解一帧。成功返回 L2 帧字节。
    func decode(pixelBuffer: CVPixelBuffer, grid n: Int32) throws -> Data {
        guard CVPixelBufferGetPixelFormatType(pixelBuffer) == kCVPixelFormatType_32BGRA else {
            throw Failure.badArgs
        }
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else {
            throw Failure.badArgs
        }
        var img = vdd_image(
            pixels: base.assumingMemoryBound(to: UInt8.self),
            width: Int32(CVPixelBufferGetWidth(pixelBuffer)),
            height: Int32(CVPixelBufferGetHeight(pixelBuffer)),
            stride: CVPixelBufferGetBytesPerRow(pixelBuffer),
            is_bgra: 1)
        return try run(&img, grid: n)
    }

    /// 从紧凑 RGB24 缓冲解一帧（测试与金图像回归用）
    func decode(rgb: [UInt8], width: Int, height: Int, grid n: Int32) throws -> Data {
        var out: Data?
        var thrown: Failure?
        rgb.withUnsafeBufferPointer { buf in
            var img = vdd_image(
                pixels: buf.baseAddress,
                width: Int32(width),
                height: Int32(height),
                stride: width * 3,
                is_bgra: 0)
            do {
                out = try run(&img, grid: n)
            } catch let e as Failure {
                thrown = e
            } catch {
                thrown = .badArgs
            }
        }
        if let e = thrown { throw e }
        return out!
    }

    private func run(_ img: inout vdd_image, grid n: Int32) throws -> Data {
        var diag = vdd_diag()
        var ret = frameBuf.withUnsafeMutableBufferPointer { out in
            vdd_decode_diag(&img, n, &lock, out.baseAddress, out.count, &diag)
        }
        /*
         * 跟踪态纠错失败就换个角点起点重来一次。跟踪态只在上一帧四角附近开小窗
         * 精定位，手持晃动下这个起点会偏，格数越多每格像素越少，同样的角点误差
         * 占比越大——真机 N=112 的诊断包里有三帧标记为纠错失败，同样的像素换个
         * 起点就是 18/18 整帧解出。
         *
         * 重试**只放大搜索窗，不丢锁**。早先这里写的是 lock.valid = 0，副作用
         * 远比想象的大：丢锁会连带触发全图搜索（4K 下 65 ms）与 refine_beta
         * 重算（56 ms），而且重试若也失败，锁就一直是丢的，接下来每一帧都得付
         * 同样的代价。真机表现是「对着静止画面 20 fps，一开始播放掉到 5 fps」——
         * 差的不是解码，是失败帧引发的连锁重定位。
         *
         * 只认 VDD_ERR_RS。信噪比过低的帧走 VDD_ERR_SIGNAL，那类帧是屏幕换帧
         * 过渡期抓到的，数据区被新旧两帧混合污染，换个定位起点结果一样，
         * 重试纯属白烧——4K 那一轮 11 个失败帧里有 5 个属于这一类。
         * VDD_DUPLICATE 同样不重试，它不是失败。
         */
        if ret == VDD_ERR_RS && lock.valid != 0 {
            lock.wide = 1
            ret = frameBuf.withUnsafeMutableBufferPointer { out in
                vdd_decode_diag(&img, n, &lock, out.baseAddress, out.count, &diag)
            }
        }
        /* 失败时 diag 也带回了走得最远那个候选的测量值，现场排查要靠它 */
        diagnostics = diag
        /*
         * 重复帧不是失败：同一屏幕帧被连续扫到多次是常态（真机 N=112 遥测里
         * 51% 的成功帧是重复）。C 核心只采了约 1/16 的格就认出来了，这里交还
         * 上一帧的字节，对调用方而言与完整解出一次完全等价——计入成功、不触发
         * 重试、照常走哈希去重。lastFrame 理论上不会为空（C 侧只有成功过才有
         * 比对记录），万一为空按纠错失败处理，不编造数据。
         */
        lastWasDuplicate = ret == VDD_DUPLICATE
        if lastWasDuplicate {
            guard let prev = lastFrame else { throw Failure.rsUncorrectable }
            return prev
        }
        if ret < 0 {
            throw Failure(rawValue: ret) ?? .badArgs
        }
        let frame = Data(frameBuf[0..<Int(ret)])
        lastFrame = frame
        return frame
    }
}
