//
//  DenseCameraScanner.swift
//  QrBinary
//
//  VD-D1 密集彩色 L1 的相机采集管线。与二维码路径的 CameraScanner 完全独立：
//  物理层不同，采集配置与处理节奏的取舍也不同，混在一起只会互相牵制。
//
//  与 QR 路径的三点关键差异：
//
//  1. **像素格式用 BGRA 而非 YUV**。多电平判决要读三个通道，YUV→BGRA 由 ISP
//     硬件完成，比在 CPU 上转换便宜得多。QR 路径只需亮度，用 YUV 更省。
//
//  2. **锁定 AE / AWB，但对焦始终自动**。屏摄的红利是屏幕本身静止，一旦解出
//     第一帧就说明曝光与白平衡够用，锁死可消除光度漂移——多电平判决最怕这个，
//     QR 是二值判决则不敏感。但**对焦必须留给相机**：手持时距离一直在变，
//     锁焦等于把可用距离钉死在一个点上（早先的 bug，见 lockExposureAndWhiteBalance）。
//     锁定后连续失败约 1 秒会自动放开，重新自动调节。
//
//  3. **不做 ML 推理**，整条链路是确定性数值计算，因此不需要 QR 路径那种
//     为保护 Vision 而设的送检帧率上限。处理成本由 C 核心的实测决定
//     （跟踪态 N=96 约 0.5ms），瓶颈在采集而非计算。
//

import AVFoundation
import Combine
import CoreVideo
import Foundation
import QuartzCore

/// 解码侧状态。`decoder` 只在串行的 decodeQueue 上访问；`grid` 会被界面改写，
/// 用锁保护。分出来是为了让 captureOutput 能安全地 nonisolated 执行——
/// 它跑在 decodeQueue 上，不是主 actor。
private final class DecodeBox: @unchecked Sendable {
    /// 仅在 decodeQueue（串行）上访问
    let decoder = DenseDecoder()
    /// 仅在 sessionQueue（串行）上访问
    var device: AVCaptureDevice?
    private let lock = NSLock()
    private var _grid: Int32 = DenseDecoder.gridAuto
    private var _diagEnabled = false
    private var _saveShots = false
    private var _wantDump = false
    private var _dumpURL: URL?

    /// 界面写、decodeQueue 读，加锁
    var grid: Int32 {
        get { lock.lock(); defer { lock.unlock() }; return _grid }
        set { lock.lock(); _grid = newValue; lock.unlock() }
    }

    /// 诊断模式开关。界面写、decodeQueue 与主 actor 读，加锁
    var diagEnabled: Bool {
        get { lock.lock(); defer { lock.unlock() }; return _diagEnabled }
        set { lock.lock(); _diagEnabled = newValue; lock.unlock() }
    }

    /// 失败帧留样开关。界面写、decodeQueue 读，加锁
    var saveShots: Bool {
        get { lock.lock(); defer { lock.unlock() }; return _saveShots }
        set { lock.lock(); _saveShots = newValue; lock.unlock() }
    }

    /// 请求把下一帧原始像素落盘（现场诊断用）
    func requestDump() { lock.lock(); _wantDump = true; lock.unlock() }

    /*
     * 失败帧采样闸门。
     *
     * CSV 只有数值，复现不了故障——要判断「几何没对上」还是「对焦糊了」，
     * 最终还得看那一帧的像素。但 1080p 的 24 位 BMP 一张约 6 MB，
     * 每个失败帧都存会瞬间吃掉磁盘并把采集队列拖垮。
     *
     * 两道闸门：整场扫描最多 10 张，且两张之间至少隔 0.5 秒。
     * 间隔的意义不只是省开销——连续帧长得几乎一样，攒 10 张连拍毫无信息量，
     * 隔开才可能覆盖到不同的失败姿态。
     */
    private var _failShots = 0
    private var _lastFailShotAt: CFTimeInterval = -1e9
    private var _scanStart: CFTimeInterval = 0

    /// 扫描起点，与 CSV 的 t_ms 同源，便于按时间戳把图和日志行对上
    var scanStart: CFTimeInterval {
        get { lock.lock(); defer { lock.unlock() }; return _scanStart }
        set { lock.lock(); _scanStart = newValue; lock.unlock() }
    }

    func claimFailureShot(at now: CFTimeInterval, limit: Int, minGap: CFTimeInterval) -> Int? {
        lock.lock()
        defer { lock.unlock() }
        guard _failShots < limit, now - _lastFailShotAt >= minGap else { return nil }
        _failShots += 1
        _lastFailShotAt = now
        return _failShots
    }

    /// 已留样张数，只读
    var failureShotCount: Int {
        lock.lock(); defer { lock.unlock() }
        return _failShots
    }

    func resetFailureShots() {
        lock.lock()
        _failShots = 0
        _lastFailShotAt = -1e9
        lock.unlock()
    }

    func takeDumpRequest() -> Bool {
        lock.lock(); defer { lock.unlock() }
        let v = _wantDump
        _wantDump = false
        return v
    }

    var dumpURL: URL? {
        get { lock.lock(); defer { lock.unlock() }; return _dumpURL }
        set { lock.lock(); _dumpURL = newValue; lock.unlock() }
    }
}

/*
 * 进程级资源占用采样。
 *
 * GPU 占用没有公开 API 可读（Metal 只能报自家的显存分配，而这条管线根本
 * 不走 GPU——YUV->BGRA 是 ISP 硬件做的，解码全在 CPU）。与其编一个数出来，
 * 不如报真正对应「发热」这个症状的东西：热状态。系统按它决定降频，
 * 它一跳到 serious/critical，帧率下滑就有了解释。
 *
 * 三个读数都不便宜（task_threads 要分配一次端口数组），因此按秒采样，
 * 逐帧复用上一次的结果。
 */
private struct SystemMetrics {
    var cpuPercent: Double = 0      // 本进程所有线程合计，可超过 100
    var memoryMB: Double = 0        // phys_footprint，与 Xcode 内存表读数同源
    var thermal: Int = 0            // ProcessInfo.ThermalState 的 rawValue

    static func sample() -> SystemMetrics {
        var m = SystemMetrics()

        var threadList: thread_act_array_t?
        var threadCount: mach_msg_type_number_t = 0
        if task_threads(mach_task_self_, &threadList, &threadCount) == KERN_SUCCESS,
           let threads = threadList {
            for i in 0..<Int(threadCount) {
                var info = thread_basic_info()
                /* THREAD_BASIC_INFO_COUNT 是 C 宏，Swift 里看不到，按结构体大小自己算 */
                var count = mach_msg_type_number_t(
                    MemoryLayout<thread_basic_info>.size / MemoryLayout<integer_t>.size)
                let ok = withUnsafeMutablePointer(to: &info) {
                    $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                        thread_info(threads[i], thread_flavor_t(THREAD_BASIC_INFO), $0, &count)
                    }
                }
                if ok == KERN_SUCCESS, info.flags & TH_FLAGS_IDLE == 0 {
                    m.cpuPercent += Double(info.cpu_usage) / Double(TH_USAGE_SCALE) * 100
                }
            }
            vm_deallocate(mach_task_self_, vm_address_t(UInt(bitPattern: threads)),
                          vm_size_t(Int(threadCount) * MemoryLayout<thread_t>.size))
        }

        var vmInfo = task_vm_info_data_t()
        var vmCount = mach_msg_type_number_t(MemoryLayout<task_vm_info_data_t>.size / MemoryLayout<integer_t>.size)
        let vmOK = withUnsafeMutablePointer(to: &vmInfo) {
            $0.withMemoryRebound(to: integer_t.self, capacity: Int(vmCount)) {
                task_info(mach_task_self_, task_flavor_t(TASK_VM_INFO), $0, &vmCount)
            }
        }
        if vmOK == KERN_SUCCESS {
            m.memoryMB = Double(vmInfo.phys_footprint) / (1024 * 1024)
        }

        m.thermal = ProcessInfo.processInfo.thermalState.rawValue
        return m
    }
}

/*
 * 扫描遥测日志。
 *
 * 真机上「纠错失败」占了失败帧的大头，但一个 VDD_ERR_RS 说明不了任何事：
 * 到底是几何没对准、对比度不够、还是只差一两个字节？逐帧把 vdd_diag 全部
 * 测量值连同结果记下来，导出成 CSV 丢进表格看趋势，才有可能定位。
 *
 * 写入发生在主 actor 的 finish 上，导出发生在界面动作上，但仍用 NSLock
 * 保护（同 DecodeBox 的做法）：这样以后把写入挪回 decodeQueue 也不必改。
 *
 * 用 CSV 而非 JSON：字段是定长的一张表，CSV 可以直接拖进表格软件画图。
 */
private final class TelemetryLog: @unchecked Sendable {

    /// 一帧一条
    struct Record {
        var tMs: Int              // 相对扫描开始的单调毫秒
        var result: String        // "ok" 或失败短名
        var diag: vdd_diag
        var frameBytes: Int       // 解出的 L2 帧字节数，失败为 0
        var duplicate: Bool       // 是否被判为重复帧
        var tracking: Bool        // 解码时是否处于跟踪锁定态
        var exposureLocked: Bool  // 曝光/白平衡是否已锁
        /*
         * 解码本身的耗时。没有它就分不清「送检帧率低」是解码跟不上还是
         * 单纯在等下一帧——4K 那一轮两种解释都能套上观测到的 ~100ms 间隔，
         * 只能靠这一列定案。
         */
        var decodeMs: Double
        /*
         * 采集时间戳到进入解码队列的延迟。它和 decodeMs 分开才说明问题：
         * 前者大说明采集管线或队列积压，后者大才是解码本身慢。
         */
        var latencyMs: Double
        /// 解码完成到主 actor 真正执行 finish 的排队时长
        var queueMs: Double
        var metrics: SystemMetrics
    }

    /// 条数上限。60fps 扫五分钟约 18000 条，再多也没有分析价值，
    /// 超出丢弃最旧的，避免长时间扫描把内存吃光。
    static let capacity = 20000
    /// 超限时一次丢一批而不是一条：Array.removeFirst(1) 是 O(n) 搬移，
    /// 每帧都做等于持续搬几 MB；成批丢摊薄到几乎为零。
    private static let dropChunk = 1024

    private let lock = NSLock()
    private var records: [Record] = []

    func append(_ r: Record) {
        lock.lock()
        defer { lock.unlock() }
        records.append(r)
        if records.count > Self.capacity {
            records.removeFirst(Self.dropChunk)
        }
    }

    func reset() {
        lock.lock()
        records.removeAll(keepingCapacity: true)
        lock.unlock()
    }

    var count: Int {
        lock.lock(); defer { lock.unlock() }
        return records.count
    }

    /// 首行表头 + 每帧一行
    func makeCSV() -> String {
        lock.lock()
        let snapshot = records
        lock.unlock()

        var s = "t_ms,result,grid,cell_px,beta,thr_r,thr_g,thr_b,"
        s += "halfgap_r,halfgap_g,halfgap_b,bow,header_disagree,payload_len,"
        s += "rs_blocks,rs_blocks_ok,rs_corrected,rs_erasures,"
        s += "frame_bytes,duplicate,tracking,exposure_locked,"
        s += "latency_ms,decode_ms,queue_ms,"
        s += "ms_find,ms_probe,ms_beta,ms_sample,ms_decide,ms_rs,ms_geom,ms_calib,"
        /* 抽样早退与分区修正的观测量，用来复核坏帧（20%）与重复（25%）两道门限 */
        s += "probe_cells,probe_low,probe_diff,zones,colors,"
        s += "cpu_pct,mem_mb,thermal\n"
        s.reserveCapacity(snapshot.count * 128)
        for r in snapshot {
            let d = r.diag
            s += "\(r.tMs),\(r.result),\(d.grid),"
            s += String(format: "%.3f,%.4f,", d.cell_px, d.beta)
            s += String(format: "%.1f,%.1f,%.1f,", d.thr.0, d.thr.1, d.thr.2)
            s += String(format: "%.1f,%.1f,%.1f,", d.halfgap.0, d.halfgap.1, d.halfgap.2)
            s += String(format: "%.4f,", d.bow)
            s += "\(d.header_disagree),\(d.payload_len),"
            s += "\(d.rs_blocks),\(d.rs_blocks_ok),\(d.rs_corrected),\(d.rs_erasures),"
            s += "\(r.frameBytes),\(r.duplicate ? 1 : 0),"
            s += "\(r.tracking ? 1 : 0),\(r.exposureLocked ? 1 : 0),"
            s += String(format: "%.1f,%.1f,%.1f,", r.latencyMs, r.decodeMs, r.queueMs)
            s += String(format: "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",
                        d.ms_find, d.ms_probe, d.ms_beta, d.ms_sample,
                        d.ms_decide, d.ms_rs, d.ms_geom, d.ms_calib)
            s += "\(d.probe_cells),\(d.probe_low),\(d.probe_diff),\(d.zones),\(d.colors),"
            s += String(format: "%.1f,%.1f,%d\n",
                        r.metrics.cpuPercent, r.metrics.memoryMB, r.metrics.thermal)
        }
        return s
    }
}

/// 整场扫描最多留几张失败帧，以及两张之间的最小间隔
private let kMaxFailureShots = 10
private let kFailureShotGap: CFTimeInterval = 0.5

/// 失败帧与遥测 CSV 的落盘目录（导出时整个打包）
private func denseDiagDir() -> URL {
    FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        .appendingPathComponent("vdd-diag", isDirectory: true)
}

/*
 * 采集缓冲的一份拷贝。留样落盘不能在 decodeQueue 上做完：4K 一帧是 8.3M
 * 像素，逐像素转 BMP 再写 24 MB 文件在真机上要一秒以上，而这条队列同时
 * 是解码路径——诊断包里每个失败帧后面都跟着 1~2 秒的空档，送检帧率被自己
 * 的诊断代码拖到 2.4 fps。CVPixelBuffer 归采集池所有，不能跨帧持有，
 * 所以先按行 memcpy 拷出来（33 MB 的 memcpy 是毫秒级），再交给后台队列。
 */
private struct FrameCopy {
    let bytes: [UInt8]
    let width: Int
    let height: Int
    let stride: Int
}

private func copyBGRA(_ pixelBuffer: CVPixelBuffer) -> FrameCopy? {
    CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
    guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }
    let w = CVPixelBufferGetWidth(pixelBuffer)
    let h = CVPixelBufferGetHeight(pixelBuffer)
    let stride = CVPixelBufferGetBytesPerRow(pixelBuffer)
    var bytes = [UInt8](repeating: 0, count: stride * h)
    bytes.withUnsafeMutableBytes { dst in
        _ = memcpy(dst.baseAddress!, base, stride * h)
    }
    return FrameCopy(bytes: bytes, width: w, height: h, stride: stride)
}

/// 把 BGRA 缓冲写成 24 位 BMP。用 BMP 而非 PNG：格式极简、无压缩、
/// 与 C 侧 harness 直接兼容，导出的帧可以原样喂给 vd_dense_core 复现问题。
private func writeBMP(_ frame: FrameCopy, to url: URL) -> Bool {
    let w = frame.width, h = frame.height, stride = frame.stride
    return frame.bytes.withUnsafeBufferPointer { buf -> Bool in
    let src = buf.baseAddress!

    let rowBytes = (w * 3 + 3) / 4 * 4
    var data = Data(count: 54 + rowBytes * h)
    data.withUnsafeMutableBytes { (raw: UnsafeMutableRawBufferPointer) in
        let p = raw.bindMemory(to: UInt8.self).baseAddress!
        func le32(_ off: Int, _ v: Int) {
            p[off] = UInt8(v & 0xFF); p[off + 1] = UInt8((v >> 8) & 0xFF)
            p[off + 2] = UInt8((v >> 16) & 0xFF); p[off + 3] = UInt8((v >> 24) & 0xFF)
        }
        p[0] = 0x42; p[1] = 0x4D                       // "BM"
        le32(2, 54 + rowBytes * h); le32(10, 54); le32(14, 40)
        le32(18, w); le32(22, h)
        p[26] = 1; p[28] = 24
        le32(34, rowBytes * h)
        for y in 0..<h {
            let s = src + (h - 1 - y) * stride        // BMP 自底向上
            let d = p + 54 + y * rowBytes
            for x in 0..<w {
                d[x * 3] = s[x * 4]                   // B
                d[x * 3 + 1] = s[x * 4 + 1]           // G
                d[x * 3 + 2] = s[x * 4 + 2]           // R
            }
        }
    }
    return (try? data.write(to: url)) != nil
    }
}

@MainActor
final class DenseCameraScanner: NSObject, ObservableObject,
                                AVCaptureVideoDataOutputSampleBufferDelegate {

    /// 解出的 L2 帧字节；交给 FileReceiver.processPayload 即可
    var onFrameDecoded: ((Data) -> Void)?

    /// 档位（符号边长，格）。默认 `DenseDecoder.gridAuto` 自动判别，
    /// 不需要与发送端手工对齐。
    var grid: Int32 {
        get { box.grid }
        set { box.grid = newValue }
    }
    var resolution: ScanResolution = .hd1080p
    /*
     * 诊断模式。关闭时逐帧遥测与失败帧留样全部跳过——这两项在 4K 下都不便宜：
     * 遥测每帧一次加锁 + 一条 Record（含整个 vdd_diag）入数组，留样一次要
     * memcpy 33 MB 再转 24 MB BMP。常规传输不该付这个钱。
     */
    var diagnosticsEnabled: Bool {
        get { box.diagEnabled }
        set { box.diagEnabled = newValue }
    }
    /// 是否留存失败帧原图。仅在诊断模式下有意义，代价见 FileReceiver 的说明
    var saveFailureFrames: Bool {
        get { box.saveShots }
        set { box.saveShots = newValue }
    }

    /// 实际判别出的档位（0 = 尚未解出任何帧）
    @Published private(set) var detectedGrid: Int32 = 0
    /// 最近一次解码的测量值，供现场标定
    @Published private(set) var diagText: String = ""
    /// 已落盘的诊断帧；导出后可直接喂给 C 侧 harness 复现
    @Published private(set) var dumpedFrameURL: URL?

    /// 请求把下一帧原始像素落盘
    func dumpNextFrame() {
        dumpedFrameURL = nil
        box.requestDump()
    }

    /// 送检帧率与命中帧率，语义同 QR 路径
    let processedMeter = ThroughputMeter()
    let detectedMeter = ThroughputMeter()

    /// 原始速率：每秒解出的 L2 帧字节数，含喷泉码冗余与重复帧。
    /// 反映物理层的实际吞吐能力。
    let rawByteMeter = ThroughputMeter()
    /// 有效速率：去重后真正新增的字节数，即真正推进传输的部分。
    /// 两者的比值就是当前取景下的重复率。
    let netByteMeter = ThroughputMeter()

    /// 已落盘的扫描遥测 CSV
    @Published private(set) var exportedLogURL: URL?
    /// 已记录的遥测条数，供界面显示。
    /// 刻意不做 @Published：每帧都发一次变更会把 SwiftUI 的重绘拉到 30Hz，
    /// 而这个数只在打开菜单时看一眼，随界面自然重绘刷新即可。
    var logCount: Int { telemetry.count }
    /// 本场已留样的失败帧张数。同样不做 @Published，理由同上
    var failureShotCount: Int { box.failureShotCount }

    private let telemetry = TelemetryLog()
    /// 扫描开始时刻（单调时钟），遥测时间戳的基准
    private var scanStart: CFTimeInterval = CACurrentMediaTime()

    /*
     * 导出诊断包：遥测 CSV + 本场留样的失败帧，打成一个 zip。
     *
     * 分开导出要操作十来次分享面板，实际用起来没人愿意做。
     * zip 用 NSFileCoordinator 的 .forUploading——系统自带，不引第三方依赖。
     */
    func exportTelemetry() {
        exportedLogURL = nil
        let dir = denseDiagDir()
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        /* CSV 与失败帧放同一个目录，一起打包 */
        let csv = telemetry.makeCSV()
        try? csv.write(to: dir.appendingPathComponent("scanlog.csv"),
                       atomically: true, encoding: .utf8)

        let stamp = Int(Date().timeIntervalSince1970)
        let dest = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("vdd-diag-\(stamp).zip")
        try? FileManager.default.removeItem(at: dest)

        var coordErr: NSError?
        var ok = false
        NSFileCoordinator().coordinate(readingItemAt: dir, options: .forUploading,
                                       error: &coordErr) { zipURL in
            ok = (try? FileManager.default.copyItem(at: zipURL, to: dest)) != nil
        }
        guard ok, coordErr == nil else { return }
        exportedLogURL = dest
    }

    /// 最近一次失败原因，供界面提示对焦/取景
    @Published private(set) var lastFailure: String = ""
    /// 曝光与白平衡是否已锁定
    @Published private(set) var isLocked = false

    let session = AVCaptureSession()

    private let videoOutput = AVCaptureVideoDataOutput()
    private let sessionQueue = DispatchQueue(label: "dense.camera.session")
    /// 串行队列 + alwaysDiscardsLateVideoFrames 已实现「忙时丢帧」，
    /// 不需要额外的 busy 标志。喷泉码下丢帧只是线性代价。
    private let decodeQueue = DispatchQueue(label: "dense.decode", qos: .userInitiated)
    /// 留样落盘队列。串行 + 低优先级，绝不和解码抢 CPU
    private let dumpQueue = DispatchQueue(label: "dense.dump", qos: .utility)
    /// 资源占用按秒采样，逐帧复用
    private var metricsCache = SystemMetrics()
    private var metricsAt: CFTimeInterval = 0
    private let box = DecodeBox()
    private var isRunning = false
    /// 连续解码成功次数，达到阈值即锁定 AE/AWB
    private var successStreak = 0
    private var failStreak = 0
    /// 已交付过的帧哈希，避免同一符号被反复送进解码器
    private var recentHashes = Set<Int>()

    private static let lockAfterSuccesses = 3
    /* 采集帧率上限，见 configure */
    nonisolated private static let captureFps = 60
    /* 约 1 秒的连续失败（60fps）即放开曝光与对焦锁：短于此会被偶发丢帧误触发 */
    private static let unlockAfterFailures = 60
    /*
     * 连续失败到这个数就先补一次单次对焦。真机上「突然一直解不出」多数是失焦，
     * 而不是取景不对；主动触发一次搜索比干等持续自动对焦自己察觉快得多。
     * 取值明显小于解锁阈值：先试成本低的（重新合焦），不行再退回全自动。
     */
    private static let refocusAfterFailures = 16

    func start() {
        guard !isRunning else { return }
        isRunning = true
        processedMeter.reset()
        detectedMeter.reset()
        rawByteMeter.reset()
        netByteMeter.reset()
        telemetry.reset()
        scanStart = CACurrentMediaTime()
        box.scanStart = scanStart
        /* 每场扫描重新计 10 张，并清掉上一场的留样，避免混在一起分析 */
        box.resetFailureShots()
        try? FileManager.default.removeItem(at: denseDiagDir())
        successStreak = 0
        box.decoder.resetLock()
        /* 2K 没有对应预设，走 .inputPriority 由 configure 直接选设备 format */
        let preset: AVCaptureSession.Preset
        switch resolution {
        case .hd720p: preset = .hd1280x720
        case .hd1080p: preset = .hd1920x1080
        case .qhd2K: preset = .inputPriority
        case .uhd4K: preset = .hd4K3840x2160
        }
        let want = resolution
        sessionQueue.async { [weak self] in
            self?.configure(preset: preset, resolution: want)
            self?.session.startRunning()
        }
    }

    func stop() {
        guard isRunning else { return }
        isRunning = false
        sessionQueue.async { [weak self] in
            self?.session.stopRunning()
            self?.unlockDevice()
        }
    }

    /// 换档或重新取景：清跟踪锁与曝光锁，回到全图搜索
    func resetTracking() {
        box.decoder.resetLock()
        successStreak = 0
        sessionQueue.async { [weak self] in self?.unlockDevice() }
        isLocked = false
    }

    /// 在 sessionQueue 上执行：只碰 session/videoOutput（AVCaptureSession 的
    /// begin/commitConfiguration 区间是线程安全的）与 box.device
    private nonisolated func configure(preset: AVCaptureSession.Preset,
                                       resolution: ScanResolution) {
        session.beginConfiguration()

        guard let dev = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back),
              let input = try? AVCaptureDeviceInput(device: dev),
              session.canAddInput(input) else {
            session.commitConfiguration()
            return
        }
        session.addInput(input)
        box.device = dev

        /* canSetSessionPreset 取决于当前输入，必须在 addInput 之后问。
           2K / 4K 并非所有机型与镜头都支持，不能设就退回 1080p */
        session.sessionPreset = session.canSetSessionPreset(preset) ? preset : .hd1920x1080

        if (try? dev.lockForConfiguration()) != nil {
            /*
             * 选 activeFormat。sessionPreset 隐含 30fps 上限，要上 60fps 只能自己挑 format
             * （设了 activeFormat，session 会自动转成 .inputPriority）。
             * - 候选只要 8-bit 4:2:0：10-bit HDR 格式会改动色调映射，调色板判决怕这个；
             * - 尺寸：720p/1080p/4K 要求宽高与预设一致；2K 没有预设，取宽度最接近目标的
             *   （1920...3200），预设设不上而退回 1080p 时按 1080p 挑；
             * - 帧率：能到 60 的取最高帧率最低的一档，避开 120/240fps 慢动作格式
             *   （多为像素合并，画质差）；都到不了 60 就取帧率最高的一档。
             * 挑不到就什么都不做，session 沿用预设自己选的那个（通常 30fps）。
             */
            let fellBack = session.sessionPreset != preset
            let target = fellBack ? 1920 : resolution.targetWidth
            let anyHeight = resolution == .qhd2K && !fellBack
            let fps = Double(Self.captureFps)
            func width(_ f: AVCaptureDevice.Format) -> Int {
                Int(CMVideoFormatDescriptionGetDimensions(f.formatDescription).width)
            }
            func rateCost(_ f: AVCaptureDevice.Format) -> Double {
                let r = f.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0
                return r >= fps ? r - fps : 1000 - r
            }
            let pick = dev.formats.filter { f in
                let sub = CMFormatDescriptionGetMediaSubType(f.formatDescription)
                guard sub == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
                        || sub == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange else { return false }
                let d = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
                if anyHeight { return d.width >= 1920 && d.width <= 3200 }
                return Int(d.width) == target && Int(d.height) == target * 9 / 16
            }.min { a, b in
                let wa = abs(width(a) - target), wb = abs(width(b) - target)
                return wa != wb ? wa < wb : rateCost(a) < rateCost(b)
            }
            if let pick { dev.activeFormat = pick }
            /*
             * 对焦策略：屏摄的被摄物是 20–60 cm 处的一块屏幕，且手持时距离一直在变。
             * - 限制对焦搜索范围到近距，避免来回拉到无穷远，合焦快得多；
             * - 对焦点钉在画面中心（符号总在取景框中央）；
             * - 关掉平滑对焦：那是为录像观感设计的，会拖慢合焦，扫码要的是快。
             */
            if dev.isFocusModeSupported(.continuousAutoFocus) {
                dev.focusMode = .continuousAutoFocus
            }
            if dev.isAutoFocusRangeRestrictionSupported {
                dev.autoFocusRangeRestriction = .near
            }
            if dev.isFocusPointOfInterestSupported {
                dev.focusPointOfInterest = CGPoint(x: 0.5, y: 0.5)
            }
            if dev.isSmoothAutoFocusSupported {
                dev.isSmoothAutoFocusEnabled = false
            }
            if dev.isExposurePointOfInterestSupported {
                dev.exposurePointOfInterest = CGPoint(x: 0.5, y: 0.5)
            }
            /*
             * 帧率上限提到 60fps（格式到不了就取它的最高帧率）。早先限 20fps 的理由是
             * 「解码慢」，但那是 -O0 Debug 构建测出的假象：C 核心 -O2 下 N=128 约 3.3ms。
             * 真机遥测在 20fps / 发送端 83ms 时稳定为 ooowwooowwooo——每个符号恰好
             * 只拿到一个干净帧，瓶颈在采集时间轴。60fps 配发送端 50ms（3 个 60Hz 刷新
             * 周期）时，每个符号有 3 次采样，换帧过渡最多污染其中 2 帧。
             *
             * 只设 MinFrameDuration（帧率上限），MaxFrameDuration 复位到格式允许的最大值，
             * 不把帧时长钉死：自动曝光期间帧时长可以浮动，采样相位随之漂移，不会与
             * 发送端锁在一个坏相位上。先复位 Max 再设 Min：AVCaptureDevice 是设备级单例，
             * 上一场扫描留下的值会一直活着。
             */
            if let range = dev.activeFormat.videoSupportedFrameRateRanges
                .max(by: { $0.maxFrameRate < $1.maxFrameRate }) {
                let wanted = CMTime(value: 1, timescale: CMTimeScale(Self.captureFps))
                dev.activeVideoMaxFrameDuration = range.maxFrameDuration
                dev.activeVideoMinFrameDuration =
                    CMTimeMaximum(CMTimeMinimum(wanted, range.maxFrameDuration), range.minFrameDuration)
            }
            dev.unlockForConfiguration()
        }

        // BGRA：YUV->RGB 交给 ISP，CPU 侧直接拿到三通道
        videoOutput.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
        ]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        videoOutput.setSampleBufferDelegate(self, queue: decodeQueue)
        if session.canAddOutput(videoOutput) {
            session.addOutput(videoOutput)
        }
        session.commitConfiguration()
    }

    /*
     * 锁定曝光与白平衡。解出若干帧说明当前参数可用，锁死以消除逐帧光度漂移。
     * 在 sessionQueue 上执行。
     *
     * **对焦绝不能一起锁。** 早先这里连 focusMode 也置成 .locked，于是解出 3 帧后
     * 焦距被冻住，距离一变就再也解不出，手持轻微前后移动即失效，
     * 表现为「只能在某个固定距离识别」。光度漂移是曝光与白平衡的问题，
     * 清晰度是对焦的问题，两者必须分开处置。
     *
     * 同时把快门时间压到 1/250 s 以内、用 ISO 补偿，超出 ISO 上限就接受欠曝。
     * 两个理由：一是手持抖动的运动模糊比噪声更致命；二是曝光越短，跨在屏幕换帧
     * 瞬间的那段时间窗越窄，过渡帧（VDD_ERR_SIGNAL）随之减少。调色板判决对整体
     * 明暗免疫——参考色和数据格在同一帧里一起变暗，最近邻判决不受影响。
     *
     * 取舍：曝光一旦固定（custom 或 locked），自动曝光不再拉伸帧时长，相机实际上
     * 按 MinFrameDuration 严格周期采样——MaxFrameDuration 留在格式最大值也挡不住。
     * 此时与发送端的相位只靠两边晶振的微小偏差漂移，可视为锁定。我们保留 custom
     * 曝光（短快门的收益是实打实的），把防线放在比例上：发送端每符号至少停留
     * 3 个采集周期（60fps 下 50ms），过渡带（读出时间 + 曝光）最多跨 2 帧，
     * 任何相位下每个符号都至少剩一个干净帧。2 个周期（33ms）在锁相下可能整符号
     * 全废，不推荐。
     */
    private static let maxExposureSeconds = 1.0 / 250.0

    private nonisolated func lockExposureAndWhiteBalance() {
        guard let dev = box.device, (try? dev.lockForConfiguration()) != nil else { return }
        defer { dev.unlockForConfiguration() }

        if dev.isWhiteBalanceModeSupported(.locked) { dev.whiteBalanceMode = .locked }

        /*
         * 连同镜头位置一起锁死。
         *
         * 这和早先那个「一开始就锁焦」的 bug 不是一回事：那次是从第一帧就把
         * 焦距钉住，手持距离一变就再也解不出。这里是**已经连续解出若干帧**之后
         * 才锁——此刻的镜头位置被实测证明可用，锁住是为了掐掉持续自动对焦的
         * 来回搜索，那个搜索过程本身会产出一串糊帧。连续失败会走 unlockDevice
         * 放开，重新自动对焦，所以换距离仍然跟得上。
         */
        if dev.isFocusModeSupported(.locked) {
            dev.setFocusModeLocked(lensPosition: dev.lensPosition, completionHandler: nil)
        }

        let cap = CMTime(seconds: Self.maxExposureSeconds, preferredTimescale: 1_000_000)
        let cur = dev.exposureDuration
        if dev.isExposureModeSupported(.custom), CMTimeCompare(cur, cap) > 0 {
            /* 缩短快门的同时按比例提 ISO，超出上限就接受欠曝 */
            let ratio = CMTimeGetSeconds(cur) / CMTimeGetSeconds(cap)
            let want = Float(Double(dev.iso) * ratio)
            let iso = min(max(want, dev.activeFormat.minISO), dev.activeFormat.maxISO)
            dev.setExposureModeCustom(duration: cap, iso: iso, completionHandler: nil)
        } else if dev.isExposureModeSupported(.locked) {
            dev.exposureMode = .locked
        }
    }

    /*
     * 强制重新合焦一次。连续失败但还没到解锁阈值时调用——多数「突然解不出」
     * 是失焦而非取景问题，主动触发一次单次自动对焦比干等持续自动对焦自己
     * 察觉要快。仅在 sessionQueue 上执行。
     */
    private nonisolated func refocus() {
        guard let dev = box.device, (try? dev.lockForConfiguration()) != nil else { return }
        defer { dev.unlockForConfiguration() }
        if dev.isFocusPointOfInterestSupported {
            dev.focusPointOfInterest = CGPoint(x: 0.5, y: 0.5)
        }
        if dev.isFocusModeSupported(.autoFocus) {
            dev.focusMode = .autoFocus            /* 单次搜索，完成后自己停住 */
        }
    }

    /// 在 sessionQueue 上执行
    private nonisolated func unlockDevice() {
        guard let dev = box.device, (try? dev.lockForConfiguration()) != nil else { return }
        if dev.isExposureModeSupported(.continuousAutoExposure) {
            dev.exposureMode = .continuousAutoExposure
        }
        if dev.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) {
            dev.whiteBalanceMode = .continuousAutoWhiteBalance
        }
        if dev.isFocusModeSupported(.continuousAutoFocus) {
            dev.focusMode = .continuousAutoFocus
        }
        dev.unlockForConfiguration()
    }

    // MARK: - AVCaptureVideoDataOutputSampleBufferDelegate

    /// 跑在 decodeQueue 上，不是主 actor——只碰 box，结果再抛回主 actor
    nonisolated func captureOutput(_ output: AVCaptureOutput,
                                   didOutput sampleBuffer: CMSampleBuffer,
                                   from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }

        /* 采集时间戳到现在，即这一帧在采集管线与队列里待了多久 */
        let pts = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        let latencyMs = pts.isValid
            ? max(0, (CACurrentMediaTime() - CMTimeGetSeconds(pts)) * 1000) : 0

        // 现场诊断：把这一帧原样落盘，可直接喂给 C 侧 harness 复现
        if box.takeDumpRequest(), let copy = copyBGRA(pixelBuffer) {
            let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            let name = "vdd-frame-\(Int(Date().timeIntervalSince1970)).bmp"
            let url = dir.appendingPathComponent(name)
            dumpQueue.async { [weak self] in
                let ok = writeBMP(copy, to: url)
                Task { @MainActor [weak self] in self?.dumpedFrameURL = ok ? url : nil }
            }
        }

        var decoded: Data?
        var failure: DenseDecoder.Failure?
        let t0 = CACurrentMediaTime()
        do {
            decoded = try box.decoder.decode(pixelBuffer: pixelBuffer, grid: box.grid)
        } catch let e as DenseDecoder.Failure {
            failure = e
        } catch {
            failure = .badArgs
        }

        let decodeMs = (CACurrentMediaTime() - t0) * 1000
        let result = decoded
        let err = failure
        let diag = box.decoder.diagnostics
        /* 跟踪态要在解码后、抛回主 actor 前读：decoder 只在 decodeQueue 上访问 */
        let tracking = box.decoder.isTracking
        /* 时间戳取在采集线程上，避免主 actor 排队时长混进读数 */
        let at = CACurrentMediaTime()

        /* 失败帧留样：CSV 只有数字，复现故障还得看像素。
           限量限频由 claimFailureShot 把关，文件名带上与 CSV 同源的 t_ms，
           便于把图和日志行对上。 */
        if box.diagEnabled, box.saveShots, let e = err,
           let n = box.claimFailureShot(at: at, limit: kMaxFailureShots,
                                        minGap: kFailureShotGap),
           let copy = copyBGRA(pixelBuffer) {
            let dir = denseDiagDir()
            let ms = Int((at - box.scanStart) * 1000)
            let name = String(format: "fail%02d-%@-%dms.bmp", n, e.logName, ms)
            dumpQueue.async {
                try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
                _ = writeBMP(copy, to: dir.appendingPathComponent(name))
            }
        }

        Task { @MainActor [weak self] in
            self?.finish(result, err, diag, tracking, at, decodeMs, latencyMs)
        }
    }

    private func finish(_ decoded: Data?, _ failure: DenseDecoder.Failure?,
                        _ diag: vdd_diag, _ tracking: Bool = false,
                        _ at: CFTimeInterval = CACurrentMediaTime(),
                        _ decodeMs: Double = 0,
                        _ latencyMs: Double = 0) {
        /* 这一跳的排队时长：解码在采集队列上结束于 at，这里才真正跑起来 */
        let queueMs = max(0, (CACurrentMediaTime() - at) * 1000)
        processedMeter.record()
        var duplicate = false
        defer {
            /* 成功与失败都记一条：只记失败无法算失败率，只记成功看不到劣化过程。
               诊断模式关闭时整条跳过。 */
            if box.diagEnabled {
                telemetry.append(TelemetryLog.Record(
                    tMs: Int((at - scanStart) * 1000),
                    result: failure?.logName ?? "ok",
                    diag: diag,
                    frameBytes: decoded?.count ?? 0,
                    duplicate: duplicate,
                    tracking: tracking,
                    exposureLocked: isLocked,
                    decodeMs: decodeMs,
                    latencyMs: latencyMs,
                    queueMs: queueMs,
                    metrics: sampledMetrics(at)))
            }
        }

        guard let frame = decoded else {
            successStreak = 0
            failStreak += 1
            /* 连续失败说明锁定时的曝光已经不适用了（换了距离、角度或环境光），
               放开让相机重新自动调节，否则会一直卡在一组过时的参数上 */
            if isLocked && failStreak >= Self.unlockAfterFailures {
                isLocked = false
                failStreak = 0
                sessionQueue.async { [weak self] in self?.unlockDevice() }
            } else if isLocked && failStreak == Self.refocusAfterFailures {
                sessionQueue.async { [weak self] in self?.refocus() }
            }
            lastFailure = Self.describe(failure)
            /* 失败时也把测得的几何与对比度显示出来——判断是几何没对上
               还是真的对比度不足，全靠这几个数 */
            if diag.grid > 0 {
                diagText = String(format: "试 %d格 %.1fpx/格 对比度 R%.0f G%.0f B%.0f",
                                  diag.grid, diag.cell_px,
                                  diag.halfgap.0, diag.halfgap.1, diag.halfgap.2)
            }
            return
        }
        detectedMeter.record()
        /* 原始速率：解出即计入，重复帧也算——这是物理层真实搬运的字节量 */
        rawByteMeter.record(bytes: frame.count)
        failStreak = 0
        lastFailure = ""
        detectedGrid = diag.grid
        diagText = String(format: "%d格 %d色 %.1fpx/格 弓曲%.2f 分歧%d",
                          diag.grid, diag.colors, diag.cell_px, diag.bow, diag.header_disagree)

        // 解出若干帧后锁定曝光/白平衡，消除后续光度漂移
        successStreak += 1
        if successStreak == Self.lockAfterSuccesses && !isLocked {
            isLocked = true
            sessionQueue.async { [weak self] in self?.lockExposureAndWhiteBalance() }
        }

        // 同一符号会被连续多帧扫到，重复帧不必再进协议栈
        guard recentHashes.insert(frame.hashValue).inserted else {
            duplicate = true
            return
        }
        if recentHashes.count > 4096 { recentHashes.removeAll(keepingCapacity: true) }
        /* 有效速率：只有过了去重的帧才算，这才是真正推进传输的字节 */
        netByteMeter.record(bytes: frame.count)
        onFrameDecoded?(frame)
    }

    /// 资源占用每秒采一次，其余帧复用上一次的读数（采样本身不便宜）
    private func sampledMetrics(_ now: CFTimeInterval) -> SystemMetrics {
        if now - metricsAt >= 1.0 {
            metricsCache = SystemMetrics.sample()
            metricsAt = now
        }
        return metricsCache
    }

    private static func describe(_ f: DenseDecoder.Failure?) -> String {
        switch f {
        case .finderNotFound: return "未找到定位图形，请让符号完整入镜"
        case .headerInvalid: return "头部校验失败，请对焦或靠近"
        case .calibrationFailed: return "定标条对比度不足，请调整亮度或反光"
        case .rsUncorrectable: return "纠错失败，画面可能过暗或抖动"
        case .signalTooWeak: return "画面信噪比过低，多半抓在屏幕换帧的瞬间"
        case .capacityMismatch: return "载荷长度异常"
        case .badArgs, .outOfMemory, .none: return ""
        }
    }
}
