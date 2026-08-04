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
    private var _wantDump = false
    private var _dumpURL: URL?

    /// 界面写、decodeQueue 读，加锁
    var grid: Int32 {
        get { lock.lock(); defer { lock.unlock() }; return _grid }
        set { lock.lock(); _grid = newValue; lock.unlock() }
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
    }

    /// 条数上限。30fps 扫十分钟约 18000 条，再多也没有分析价值，
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
        s += "frame_bytes,duplicate,tracking,exposure_locked\n"
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
            s += "\(r.tracking ? 1 : 0),\(r.exposureLocked ? 1 : 0)\n"
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

/// 把 BGRA 缓冲写成 24 位 BMP。用 BMP 而非 PNG：格式极简、无压缩、
/// 与 C 侧 harness 直接兼容，导出的帧可以原样喂给 vd_dense_core 复现问题。
private func writeBMP(pixelBuffer: CVPixelBuffer, to url: URL) -> Bool {
    CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
    guard let base = CVPixelBufferGetBaseAddress(pixelBuffer) else { return false }
    let w = CVPixelBufferGetWidth(pixelBuffer)
    let h = CVPixelBufferGetHeight(pixelBuffer)
    let stride = CVPixelBufferGetBytesPerRow(pixelBuffer)
    let src = base.assumingMemoryBound(to: UInt8.self)

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
    private let box = DecodeBox()
    private var isRunning = false
    /// 连续解码成功次数，达到阈值即锁定 AE/AWB
    private var successStreak = 0
    private var failStreak = 0
    /// 已交付过的帧哈希，避免同一符号被反复送进解码器
    private var recentHashes = Set<Int>()

    private static let lockAfterSuccesses = 3
    /* 约 1 秒的连续失败即放开曝光锁：短于此会被偶发丢帧误触发 */
    private static let unlockAfterFailures = 30

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
        let preset: AVCaptureSession.Preset
        switch resolution {
        case .hd720p: preset = .hd1280x720
        case .hd1080p: preset = .hd1920x1080
        case .uhd4K: preset = .hd4K3840x2160
        }
        sessionQueue.async { [weak self] in
            self?.configure(preset: preset)
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
    private nonisolated func configure(preset: AVCaptureSession.Preset) {
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
           4K 并非所有机型/镜头都支持，不能设就退回 1080p */
        session.sessionPreset = session.canSetSessionPreset(preset) ? preset : .hd1920x1080

        if (try? dev.lockForConfiguration()) != nil {
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
     * 同时把快门时间压到 1/120 s 以内、用 ISO 补偿：手持抖动带来的运动模糊
     * 比噪声更致命，而调色板判决对整体明暗免疫——参考色和数据格在同一帧里
     * 一起变暗，最近邻判决不受影响，所以宁可欠曝也要短曝光。
     */
    private static let maxExposureSeconds = 1.0 / 120.0

    private nonisolated func lockExposureAndWhiteBalance() {
        guard let dev = box.device, (try? dev.lockForConfiguration()) != nil else { return }
        defer { dev.unlockForConfiguration() }

        if dev.isWhiteBalanceModeSupported(.locked) { dev.whiteBalanceMode = .locked }

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

        // 现场诊断：把这一帧原样落盘，可直接喂给 C 侧 harness 复现
        if box.takeDumpRequest() {
            let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            let name = "vdd-frame-\(Int(Date().timeIntervalSince1970)).bmp"
            let url = dir.appendingPathComponent(name)
            box.dumpURL = writeBMP(pixelBuffer: pixelBuffer, to: url) ? url : nil
            let saved = box.dumpURL
            Task { @MainActor [weak self] in self?.dumpedFrameURL = saved }
        }

        var decoded: Data?
        var failure: DenseDecoder.Failure?
        do {
            decoded = try box.decoder.decode(pixelBuffer: pixelBuffer, grid: box.grid)
        } catch let e as DenseDecoder.Failure {
            failure = e
        } catch {
            failure = .badArgs
        }

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
        if let e = err,
           let n = box.claimFailureShot(at: at, limit: kMaxFailureShots,
                                        minGap: kFailureShotGap) {
            let dir = denseDiagDir()
            try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
            let ms = Int((at - box.scanStart) * 1000)
            let name = String(format: "fail%02d-%@-%dms.bmp", n, e.logName, ms)
            _ = writeBMP(pixelBuffer: pixelBuffer, to: dir.appendingPathComponent(name))
        }

        Task { @MainActor [weak self] in
            self?.finish(result, err, diag, tracking, at)
        }
    }

    private func finish(_ decoded: Data?, _ failure: DenseDecoder.Failure?,
                        _ diag: vdd_diag, _ tracking: Bool = false,
                        _ at: CFTimeInterval = CACurrentMediaTime()) {
        processedMeter.record()
        var duplicate = false
        defer {
            /* 成功与失败都记一条：只记失败无法算失败率，只记成功看不到劣化过程 */
            telemetry.append(TelemetryLog.Record(
                tMs: Int((at - scanStart) * 1000),
                result: failure?.logName ?? "ok",
                diag: diag,
                frameBytes: decoded?.count ?? 0,
                duplicate: duplicate,
                tracking: tracking,
                exposureLocked: isLocked))
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
        diagText = String(format: "%d格 %.1fpx/格 弓曲%.2f 分歧%d",
                          diag.grid, diag.cell_px, diag.bow, diag.header_disagree)

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

    private static func describe(_ f: DenseDecoder.Failure?) -> String {
        switch f {
        case .finderNotFound: return "未找到定位图形，请让符号完整入镜"
        case .headerInvalid: return "头部校验失败，请对焦或靠近"
        case .calibrationFailed: return "定标条对比度不足，请调整亮度或反光"
        case .rsUncorrectable: return "纠错失败，画面可能过暗或抖动"
        case .capacityMismatch: return "载荷长度异常"
        case .badArgs, .outOfMemory, .none: return ""
        }
    }
}
