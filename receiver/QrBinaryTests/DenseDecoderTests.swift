//
//  DenseDecoderTests.swift
//  QrBinaryTests
//
//  VD-D1 密集彩色 L1 解码回归：金图像由 protocol/dense/dense_refimpl.py 生成
//  （真实 L2 帧 + 光度/几何失真），清单 vectors/dense/manifest.txt。
//  该管线不依赖 Vision，模拟器可全流程回归——这是与 QR 路径的关键差别。
//
//  重新生成金图像：python3 protocol/dense/gen_golden.py（见仓库根 README）
//

import CoreGraphics
import Foundation
import ImageIO
import Testing
@testable import QrBinary

/// 金图像目录（含 manifest.txt）。优先级同 ProtocolVectorTests：
/// 环境变量 -> 测试 bundle -> 仅 macOS 的源码回退。
///
/// 注意：同步组把 QrBinaryTests/vectors/ 下的资源**拍平**到 bundle 根，
/// 因此 bundle 侧不存在 vectors/dense 子目录，直接找 resourceURL。
private func denseVectorsURL() -> URL? {
    let fm = FileManager.default
    if let env = ProcessInfo.processInfo.environment["VD_DENSE_GOLDEN_DIR"],
       fm.fileExists(atPath: env + "/manifest.txt") {
        return URL(fileURLWithPath: env)
    }
    let bundle = Bundle(for: VectorBundleAnchor.self)
    if let root = bundle.resourceURL {
        for candidate in [root, root.appendingPathComponent("vectors/dense")] {
            if fm.fileExists(atPath: candidate.appendingPathComponent("manifest.txt").path) {
                return candidate
            }
        }
    }
    return nil
}

/// 读 24 位 BMP 为紧凑 RGB24（自底向上转自顶向下，BGR 转 RGB）
private func readBmp(_ url: URL) throws -> (rgb: [UInt8], width: Int, height: Int) {
    let data = try Data(contentsOf: url)
    func le32(_ off: Int) -> Int {
        Int(data[off]) | Int(data[off + 1]) << 8 | Int(data[off + 2]) << 16 | Int(data[off + 3]) << 24
    }
    let w = le32(18), h = le32(22), off = le32(10)
    let rowBytes = (w * 3 + 3) / 4 * 4
    var rgb = [UInt8](repeating: 0, count: w * h * 3)
    data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
        let p = raw.bindMemory(to: UInt8.self).baseAddress!
        for y in 0..<h {
            let src = p + off + (h - 1 - y) * rowBytes
            for x in 0..<w {
                rgb[(y * w + x) * 3] = src[x * 3 + 2]
                rgb[(y * w + x) * 3 + 1] = src[x * 3 + 1]
                rgb[(y * w + x) * 3 + 2] = src[x * 3]
            }
        }
    }
    return (rgb, w, h)
}

@Suite("VD-D1 密集彩色 L1")
struct DenseDecoderTests {

    @Test("RS errata 随机回归")
    func rsSelfTest() {
        #expect(DenseDecoder.rsSelfTest(trials: 500))
    }

    @Test("容量表与规约一致")
    func capacities() {
        let expected: [Int32: Int32] = [64: 1119, 80: 1863, 96: 2767, 112: 3831, 128: 5087]
        for (n, net) in expected {
            #expect(DenseDecoder.netCapacity(grid: n) == net, "N=\(n)")
        }
        #expect(DenseDecoder.netCapacity(grid: 100) == 0)
    }

    @Test("金图像全量解码")
    func goldenImages() throws {
        guard let dir = denseVectorsURL() else {
            Issue.record("测试 bundle 缺少 vectors/dense 资源")
            return
        }
        let manifest = try String(contentsOf: dir.appendingPathComponent("manifest.txt"),
                                  encoding: .utf8)
        var cases = 0
        for line in manifest.split(separator: "\n") {
            if line.hasPrefix("#") || line.isEmpty { continue }
            let parts = line.split(separator: " ")
            guard parts.count == 3, let n = Int32(parts[1]) else { continue }
            let (rgb, w, h) = try readBmp(dir.appendingPathComponent(String(parts[0])))
            let decoder = DenseDecoder()
            let frame = try decoder.decode(rgb: rgb, width: w, height: h, grid: n)
            let expectHex = String(parts[2])
            let gotHex = frame.map { String(format: "%02x", $0) }.joined()
            #expect(gotHex == expectHex, "\(parts[0]) 帧字节不一致")
            /* 解出的帧应能过现有 L2 解析（协议栈衔接） */
            #expect(FrameParser.parse([UInt8](frame)) != nil, "\(parts[0]) L2 校验失败")
            cases += 1
        }
        #expect(cases == 8, "金图像应为 8 例，实得 \(cases)")
    }

    /// 真机首测栽在这里：发送端 128 格、接收端默认 96 格，定位图形照样找得到
    /// （它们只是四个角，与 N 无关），但单应性整体错尺度、格心全偏，
    /// 表现为「定位成功但头部校验失败」。自动判别把这个坑消掉。
    @Test("自动判别档位")
    func autoGridDetection() throws {
        guard let dir = denseVectorsURL() else {
            Issue.record("测试 bundle 缺少 vectors/dense 资源")
            return
        }
        let manifest = try String(contentsOf: dir.appendingPathComponent("manifest.txt"),
                                  encoding: .utf8)
        var cases = 0
        for line in manifest.split(separator: "\n") {
            if line.hasPrefix("#") || line.isEmpty { continue }
            let parts = line.split(separator: " ")
            guard parts.count == 3, let trueGrid = Int32(parts[1]) else { continue }
            let (rgb, w, h) = try readBmp(dir.appendingPathComponent(String(parts[0])))
            let decoder = DenseDecoder()
            // 不告诉解码器档位
            let frame = try decoder.decode(rgb: rgb, width: w, height: h,
                                           grid: DenseDecoder.gridAuto)
            #expect(decoder.diagnostics.grid == trueGrid,
                    "\(parts[0]) 判出 \(decoder.diagnostics.grid) 应为 \(trueGrid)")
            let gotHex = frame.map { String(format: "%02x", $0) }.joined()
            #expect(gotHex == String(parts[2]), "\(parts[0]) 帧字节不一致")
            cases += 1
        }
        #expect(cases == 8)
    }

    /// 真机第二个坑：符号没有朝向标记，四个定位图形长得一样，
    /// 横拍或设备旋转会让符号相对图像转 90/180/270°。此时定位全对、
    /// 单应性尺度也全对，只是行列被转置，表现同样是「头部校验失败」。
    /// 真机实测中这一条独占「能定位但头部失败」的全部案例。
    @Test("符号任意朝向都能解")
    func rotatedSymbols() throws {
        guard let dir = denseVectorsURL() else {
            Issue.record("测试 bundle 缺少 vectors/dense 资源")
            return
        }
        let (rgb, w, h) = try readBmp(dir.appendingPathComponent("g96_clean.bmp"))
        let decoder = DenseDecoder()
        let ref = try decoder.decode(rgb: rgb, width: w, height: h, grid: 96)
        for quarter in 1...3 {
            var (cur, cw, ch) = (rgb, w, h)
            for _ in 0..<quarter { (cur, cw, ch) = rotate90(cur, cw, ch) }
            let d = DenseDecoder()
            let got = try d.decode(rgb: cur, width: cw, height: ch,
                                   grid: DenseDecoder.gridAuto)
            #expect(got == ref, "旋转 \(quarter * 90)° 后帧字节不一致")
        }
    }

    /// 顺时针旋转 90°：(x,y) -> (h-1-y, x)
    private func rotate90(_ src: [UInt8], _ w: Int, _ h: Int) -> ([UInt8], Int, Int) {
        var dst = [UInt8](repeating: 0, count: w * h * 3)
        for y in 0..<h {
            for x in 0..<w {
                let nx = h - 1 - y, ny = x
                let s = (y * w + x) * 3, d = (ny * h + nx) * 3
                dst[d] = src[s]; dst[d + 1] = src[s + 1]; dst[d + 2] = src[s + 2]
            }
        }
        return (dst, h, w)
    }

    /// 指定错误档位时应明确失败，而不是解出垃圾
    @Test("错误档位不产生假阳性")
    func wrongGridFails() throws {
        guard let dir = denseVectorsURL() else { return }
        let (rgb, w, h) = try readBmp(dir.appendingPathComponent("g96_clean.bmp"))
        let decoder = DenseDecoder()
        #expect(throws: DenseDecoder.Failure.self) {
            _ = try decoder.decode(rgb: rgb, width: w, height: h, grid: 128)
        }
    }

    @Test("跟踪锁复用与失锁回退")
    func lockReuse() throws {
        guard let dir = denseVectorsURL() else {
            Issue.record("测试 bundle 缺少 vectors/dense 资源")
            return
        }
        let (rgb, w, h) = try readBmp(dir.appendingPathComponent("g96_clean.bmp"))
        let decoder = DenseDecoder()
        /* 第一帧全图搜索建立锁，第二帧走小窗路径，结果必须一致 */
        let a = try decoder.decode(rgb: rgb, width: w, height: h, grid: 96)
        let b = try decoder.decode(rgb: rgb, width: w, height: h, grid: 96)
        #expect(a == b)
        /* 全白图像应报定位失败且不崩 */
        decoder.resetLock()
        let blank = [UInt8](repeating: 255, count: 320 * 320 * 3)
        #expect(throws: DenseDecoder.Failure.self) {
            _ = try decoder.decode(rgb: blank, width: 320, height: 320, grid: 96)
        }
    }
}

// MARK: - 真机照片回归

/// 真机照片目录（仓库根 `testdata/`）。这些是 iPhone 直接拍屏幕的原图，
/// 不入测试 bundle——单张 12MP，转成 BMP 有 36 MB，装进资源不划算，
/// 且它们是「现场样本」而非协议金标准，不该和 vectors/ 混在一起。
/// 模拟器测试跑在宿主文件系统上，按源码位置直接找即可。
private func realShotsURL() -> URL? {
    let fm = FileManager.default
    if let env = ProcessInfo.processInfo.environment["VD_REALSHOT_DIR"],
       fm.fileExists(atPath: env) {
        return URL(fileURLWithPath: env)
    }
    /* #filePath = <repo>/receiver/QrBinaryTests/DenseDecoderTests.swift */
    let dir = URL(fileURLWithPath: #filePath)
        .deletingLastPathComponent()      // QrBinaryTests
        .deletingLastPathComponent()      // receiver
        .deletingLastPathComponent()      // repo 根
        .appendingPathComponent("testdata")
    return fm.fileExists(atPath: dir.path) ? dir : nil
}

/// 用 ImageIO 解 HEIC/JPEG/PNG 为紧凑 RGB24。
///
/// 刻意**不**应用 EXIF 方向：相机管线拿到的 CVPixelBuffer 同样是未旋转的裸像素，
/// 保留原始朝向才是真机路径的等价物——朝向正是本组样本要考验的东西。
private func readImageRGB(_ url: URL) -> (rgb: [UInt8], width: Int, height: Int)? {
    guard let src = CGImageSourceCreateWithURL(url as CFURL, nil),
          let img = CGImageSourceCreateImageAtIndex(src, 0, nil) else { return nil }
    let w = img.width, h = img.height
    var rgba = [UInt8](repeating: 0, count: w * h * 4)
    let ok = rgba.withUnsafeMutableBytes { buf -> Bool in
        guard let ctx = CGContext(
            data: buf.baseAddress, width: w, height: h, bitsPerComponent: 8,
            bytesPerRow: w * 4, space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue) else { return false }
        ctx.draw(img, in: CGRect(x: 0, y: 0, width: w, height: h))
        return true
    }
    guard ok else { return nil }
    var rgb = [UInt8](repeating: 0, count: w * h * 3)
    for i in 0..<(w * h) {
        rgb[i * 3] = rgba[i * 4]
        rgb[i * 3 + 1] = rgba[i * 4 + 1]
        rgb[i * 3 + 2] = rgba[i * 4 + 2]
    }
    return (rgb, w, h)
}

/// 真机屏摄照片回归。合成失真 ≠ 真实屏摄信道，金图像全过并不代表真机能解——
/// 「朝向未处理」「refine_edges 产生负收益」这两个 bug 金图像一个都没暴露出来，
/// 全靠这组照片抓到。目录缺失时整组跳过，不影响他人克隆后跑测试。
@Suite("VD-D1 真机屏摄照片")
struct DenseRealShotTests {

    /// 文件名 -> (期望档位, 期望帧字节数)。六张都来自同一次发送，
    /// 因此解出的 L2 帧 sessionId 必须完全一致——这条比单帧解出更难蒙对。
    private static let expected: [(name: String, grid: Int32, bytes: Int)] = [
        ("档3 96格 1.HEIC", 96, 2460),
        ("档3 96格 2.HEIC", 96, 2460),
        ("档5 128格 1.jpg", 128, 4900),
        ("档5 128格 2.HEIC", 128, 4900),
        ("档5 128格 3.HEIC", 128, 4900),
        ("档5 128格 4.HEIC", 128, 4900)
    ]

    @Test("真机照片整帧解出")
    func realShotsDecode() throws {
        guard let dir = realShotsURL() else {
            Issue.record("找不到 testdata/，跳过真机回归")
            return
        }
        var sessionIds = Set<UInt32>()
        for c in Self.expected {
            let url = dir.appendingPathComponent(c.name)
            guard let (rgb, w, h) = readImageRGB(url) else {
                Issue.record("\(c.name) 读取失败")
                continue
            }
            let decoder = DenseDecoder()
            do {
                let frame = try decoder.decode(rgb: rgb, width: w, height: h,
                                               grid: DenseDecoder.gridAuto)
                #expect(decoder.diagnostics.grid == c.grid,
                        "\(c.name) 判出 \(decoder.diagnostics.grid) 应为 \(c.grid)")
                #expect(frame.count == c.bytes, "\(c.name) 帧长 \(frame.count)")
                /* 解出的必须是合法 L2 帧，否则只是 RS 蒙对 */
                guard let parsed = FrameParser.parse([UInt8](frame)) else {
                    Issue.record("\(c.name) L2 校验失败")
                    continue
                }
                sessionIds.insert(parsed.sessionId)
            } catch {
                Issue.record("\(c.name) 解码失败：\(error)")
            }
        }
        #expect(sessionIds.count == 1, "六张来自同一次发送，sessionId 应唯一：\(sessionIds)")
    }

    /// 同一帧拍两张，解出的字节必须逐字节相同。
    ///
    /// 这条比「能解出」更有力：两张照片的光照、角度、噪声都不同，
    /// 结果还能完全一致，说明解码不是靠 RS 把误码勉强兜住的。
    @Test("同一帧的两张照片解出相同字节")
    func sameFrameTwice() throws {
        guard let dir = realShotsURL() else {
            Issue.record("找不到 testdata/，跳过真机回归")
            return
        }
        var frames: [Data] = []
        for name in ["档5 128格 1.jpg", "档5 128格 2.HEIC"] {
            guard let (rgb, w, h) = readImageRGB(dir.appendingPathComponent(name)) else {
                Issue.record("\(name) 读取失败")
                return
            }
            frames.append(try DenseDecoder().decode(rgb: rgb, width: w, height: h,
                                                    grid: DenseDecoder.gridAuto))
        }
        #expect(frames[0] == frames[1], "同一帧的两张照片解出的字节不一致")
    }
}
