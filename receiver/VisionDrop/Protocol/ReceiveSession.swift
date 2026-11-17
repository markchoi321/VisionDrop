//
//  ReceiveSession.swift
//  VisionDrop
//
//  L5 会话层。对应设计文档 7.3。
//  会话参数由帧头确定且不可变；解码状态与统计随帧推进。
//

import Foundation

/// 会话统计，同时用于 11.1 识别率标定
struct SessionStats {
    var framesAccepted: Int = 0      // 通过 CRC32 的帧数
    var framesRejected: Int = 0      // 校验失败的帧数
    var blocksReceived: Int = 0      // 收到的编码块总数（含重复）
    var blocksDuplicate: Int = 0     // 重复 blockId 数
    var maxBlockId: UInt32 = 0       // 观察到的最大 blockId
    var firstSeenAt: Date = Date()
    var lastSeenAt: Date = Date()

    /// 唯一编码块数
    var blocksUnique: Int { blocksReceived - blocksDuplicate }

    /// 估算单帧成功率：接收端实收帧数 / 发送端已播帧数
    /// 发送端已播帧数由 maxBlockId 与每帧块数 m 反推
    func estimatedFrameSuccessRate(blocksPerFrame m: Int) -> Double {
        guard maxBlockId > 0, m > 0 else { return 0 }
        let sentFrames = Double(maxBlockId + 1) / Double(m)
        return sentFrames > 0 ? Double(framesAccepted) / sentFrames : 0
    }
}

/// 单个接收会话
final class ReceiveSession: Identifiable {

    // 不可变会话参数，由帧头确定
    let sessionId: UInt32
    let K: Int
    let T: Int
    let codec: Codec
    let compressed: Bool

    /*
     * 解码状态。
     *
     * **decoder 只允许在 workQueue 上访问。** 喷泉码的消元代价随秩线性增长
     * （K=2629、T=460 时秩 2000 处单块 Release 0.03 ms、Debug 9.4 ms），而一帧
     * 携带十来个块、帧以 20 fps 到达；放在主 actor 上，传输后半程必然把界面
     * 按死——进度读数和「完成」按钮一起卡住。这与快慢无关，是归属问题：
     * 界面响应和块消元是两件不相干的事，不该互相阻塞。
     *
     * 主 actor 要读的量走下面的镜像，由 workQueue 写、加锁保护。
     */
    private let decoder: BlockDecoder
    /*
     * 每个会话一条串行队列。正确性只要求「同一个会话的解码器不被并发访问」，
     * 共用一条全局队列会让互不相干的会话互相排队——两个会话同时在收时，
     * 一方的消元会把另一方按住；并行跑的测试之间也会互相干扰。
     */
    let workQueue: DispatchQueue

    private let mirrorLock = NSLock()
    private var _solved = 0
    private var _complete = false
    private var _released = false

    private func syncMirror() {          // 仅在 workQueue 上调用
        mirrorLock.lock()
        _solved = decoder.solvedCount
        _complete = decoder.isComplete
        _released = decoder.isReleased
        mirrorLock.unlock()
    }

    /// 在 workQueue 上访问解码器。调用方负责不把结果里的引用带回主 actor。
    func onWorkQueue<R>(_ body: @escaping (BlockDecoder) -> R,
                        then done: @escaping @MainActor (R) -> Void) {
        workQueue.async { [self] in
            let r = body(decoder)
            syncMirror()
            Task { @MainActor in done(r) }
        }
    }

    /// 同步版本，仅供启动时的恢复与测试使用（此时没有并发）
    @discardableResult
    func withDecoderSync<R>(_ body: (BlockDecoder) -> R) -> R {
        workQueue.sync {
            let r = body(decoder)
            syncMirror()
            return r
        }
    }

    // 元数据与统计
    var meta: StreamMeta?
    var stats: SessionStats
    /// 最近一帧的块数 m，用于估算发送端已播帧数
    var blocksPerFrame: Int = 1
    /// 完成后落盘的文件位置
    var savedFileURL: URL?
    var failureMessage: String?
    /// 正在后台还原（拼接 / 解压 / 校验 / 落盘）。期间不得再喂新块，
    /// 否则后台读取解码器状态的同时主线程在改它。
    var isFinalizing: Bool = false
    /// 还原进度 0…1，仅解方程的回代会真正推进
    var finalizeProgress: Double = 0
    /// 用户请求中止合并。后台线程轮询，故用带锁的标志而非裸 Bool
    var finalizeCancel: CancellationFlag?
    /// 「已收齐」只报一次。消元回调是异步的，收齐之后还会有若干帧的回调陆续到达
    var completionAnnounced = false
    /// 最终文件字节数。解码状态释放后由它提供展示数据
    var finalSize: Int?

    /// 已完成并落盘。此后中间态（编码块 / 源块 / 消元基）已释放，只留最终文件与统计
    var isFinished: Bool { savedFileURL != nil }
    /// 上次写进度文件的时间
    var lastSavedAt: Date = .distantPast

    var id: UInt32 { sessionId }

    init(sessionId: UInt32, K: Int, T: Int, codec: Codec, compressed: Bool,
         stats: SessionStats = SessionStats()) {
        self.sessionId = sessionId
        self.K = K
        self.T = T
        self.codec = codec
        self.compressed = compressed
        self.stats = stats
        self.workQueue = DispatchQueue(label: "vd.blocks.\(SessionId.hex(sessionId))",
                                       qos: .userInitiated)
        switch codec {
        case .linearSolve: self.decoder = LinearSolveDecoder(K: K, T: T)
        case .peeling: self.decoder = PeelingDecoder(K: K, T: T)
        }
    }

    convenience init(frame: ParsedFrame) {
        self.init(sessionId: frame.sessionId, K: frame.K, T: frame.T,
                  codec: frame.codec, compressed: frame.compressed)
        blocksPerFrame = frame.m
    }

    // MARK: - 进度

    /* 以下三个读的是镜像，不碰解码器，主 actor 上随便读 */
    var isComplete: Bool { mirrorLock.lock(); defer { mirrorLock.unlock() }; return _complete }
    var isReleased: Bool { mirrorLock.lock(); defer { mirrorLock.unlock() }; return _released }
    var solvedCount: Int { mirrorLock.lock(); defer { mirrorLock.unlock() }; return _solved }
    /// 次要指标：已解出源块（解方程方案为消元秩）占 K 的比例
    var sourceProgress: Double { K > 0 ? Double(solvedCount) / Double(K) : 0 }

    /// ε：解方程 2/K，剥洋葱 1.85/K^0.37，K ≤ 1 时取 0（CONTRACT 第 5 节）
    var epsilon: Double {
        guard K > 1 else { return 0 }
        return codec == .linearSolve ? 2.0 / Double(K) : 1.85 / pow(Double(K), 0.37)
    }

    /// 预计所需编码块数 = ceil(K × (1 + ε))
    var estimatedNeededBlocks: Int {
        max(K, Int((Double(K) * (1 + epsilon)).rounded(.up)))
    }

    /// 主进度：已接收编码块 / 预计所需
    var blockProgress: Double {
        let need = estimatedNeededBlocks
        guard need > 0 else { return 0 }
        return min(1.0, Double(stats.blocksUnique) / Double(need))
    }

    var displayName: String {
        meta?.fileName ?? "会话 \(SessionId.hex(sessionId))"
    }

    // MARK: - 帧接收

    /// 一帧消化完的结果。全是值类型，可以安全地从 workQueue 带回主 actor。
    struct IngestResult: Sendable {
        var gained = false
        var received = 0
        var duplicate = 0
        var maxBlockId: UInt32 = 0
        /// 剥洋葱方案可能在这一帧解出文件头
        var meta: StreamMeta?
        var complete = false
    }

    /*
     * 吃下一帧的全部编码块。消元在 workQueue 上做，结果回主 actor 由调用方
     * 合并进 stats——统计量归主 actor 管，解码器归 workQueue 管，两边不共享
     * 可变状态。帧序无所谓（喷泉码本来就与顺序无关），串行队列保证的是
     * 同一会话不会有两次 add 并发。
     */
    func ingest(_ frame: ParsedFrame, completion: @escaping @MainActor (IngestResult) -> Void) {
        blocksPerFrame = frame.m
        stats.framesAccepted += 1
        stats.lastSeenAt = Date()

        let m = frame.m
        let ids = (0..<m).map { frame.blockId($0) }
        let blocks = (0..<m).map { Array(frame.block($0)) }
        let wantMeta = (meta == nil)

        workQueue.async { [self] in
            var r = IngestResult()
            for i in 0..<m {
                r.received += 1
                if ids[i] > r.maxBlockId { r.maxBlockId = ids[i] }
                if decoder.add(blockId: ids[i], payload: blocks[i]) {
                    r.gained = true
                } else {
                    r.duplicate += 1
                }
            }
            if r.gained && wantMeta { r.meta = extractMetaOnWorkQueue() }
            syncMirror()
            r.complete = decoder.isComplete
            Task { @MainActor in completion(r) }
        }
    }

    /*
     * 同步版本，仅供测试。真实路径一律走上面的异步版——测试要的是确定性，
     * 为它引入等待反而会让用例变脆。
     */
    func ingestSyncForTests(_ frame: ParsedFrame) {
        blocksPerFrame = frame.m
        stats.framesAccepted += 1
        stats.lastSeenAt = Date()
        var r = IngestResult()
        let wantMeta = (meta == nil)
        withDecoderSync { d in
            for i in 0..<frame.m {
                let bid = frame.blockId(i)
                r.received += 1
                if bid > r.maxBlockId { r.maxBlockId = bid }
                if d.add(blockId: bid, payload: frame.block(i)) { r.gained = true }
                else { r.duplicate += 1 }
            }
            if r.gained && wantMeta { r.meta = extractMetaOnWorkQueue() }
        }
        apply(r)
    }

    /// 把 IngestResult 并回统计量。仅主 actor 调用。
    func apply(_ r: IngestResult) {
        stats.blocksReceived += r.received
        stats.blocksDuplicate += r.duplicate
        if r.maxBlockId > stats.maxBlockId { stats.maxBlockId = r.maxBlockId }
        if let m = r.meta, meta == nil { meta = m }
    }

    /*
     * 解出块 0 起的连续源块后即可解析 StreamHeader。
     *
     * 剥洋葱方案边收边解，源块是现成的，凑够头部长度就能读出文件名。
     * 解方程方案不行：源块要等回代才有，而回代是 O(K^2) 次块异或，1 MB 的
     * 文件 K=2098、Release 约 26 ms、Debug 达 6 秒。这个函数跑在主 actor 上，
     * 曾经在这里直接调 assemble()——于是「刚好收齐最后一块」的那一帧整个界面
     * 冻住数秒，而且这份回代结果转手就扔了，用户点合并时还要再算一遍。
     *
     * 现在解方程方案在这里什么都不做，文件名等合并时由 finalizeContent 填。
     * 收齐后不自动合并本来就是既定取舍（合并时长不可预测，交给用户触发），
     * 为了早几秒显示一个文件名而在主线程上跑一次完整回代，方向就错了。
     */
    /// 走 workQueue 取一次文件头。仅在没有并发时使用（恢复进度后、测试）。
    func tryExtractMeta() {
        guard meta == nil else { return }
        meta = withDecoderSync { _ in extractMetaOnWorkQueue() }
    }

    private func extractMetaOnWorkQueue() -> StreamMeta? {
        guard let peeling = decoder as? PeelingDecoder else { return nil }
        var buf = [UInt8]()
        var i = 0
        // 连续块凑够 54 + fileNameLen 即可解析出头部
        while i < K, let blk = peeling.solvedBlock(i) {
            buf.append(contentsOf: blk)
            i += 1
            if buf.count >= StreamCodec.headerFixedLength {
                if let m = try? StreamCodec.parseHeader(buf) { return m }
            }
        }
        return nil
    }

    // MARK: - 完成

    /// 回代。必须在 workQueue 上调用（completeSession 已经把整段放在那条队列上）。
    func assembleOnWorkQueue(progress: ((Double) -> Void)?,
                             isCancelled: () -> Bool) -> [UInt8]? {
        decoder.assemble(progress: progress, isCancelled: isCancelled)
    }

    /// 释放中间态。走 workQueue 保证不与消元并发。
    func releaseStorage() {
        workQueue.async { [self] in
            decoder.releaseStorage()
            syncMirror()
        }
    }

    /// 恢复进度文件。仅启动时调用，此时尚无并发。
    @discardableResult
    func restoreProgress(solved: [SolvedRecord], pending: [PendingRecord]) -> Int {
        withDecoderSync { $0.restore(solved: solved, pending: pending) }
    }

    /// 取进度快照。在 workQueue 上执行，结果交回调用方。
    func snapshotForSave(_ done: @escaping @Sendable ([SolvedRecord], [PendingRecord]) -> Void) {
        workQueue.async { [self] in
            done(decoder.snapshotSolved(), decoder.snapshotPending())
        }
    }

    /// 拼接源块 -> parseStream -> 解压 -> 校验 SHA-256。必须在 workQueue 上调用。
    func finalizeContent() throws -> (meta: StreamMeta, content: [UInt8]) {
        let assembled = decoder.assemble()
        let parsed = try StreamCodec.parseStream(assembled)
        guard StreamCodec.sha256(parsed.content) == parsed.meta.sha256 else {
            throw StreamError.digestMismatch
        }
        meta = parsed.meta
        return parsed
    }
}
