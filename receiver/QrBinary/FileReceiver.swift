//
//  FileReceiver.swift
//  QrBinary
//
//  Created by Victor on 2026/2/21.
//

import Foundation
import Combine
import SwiftUI

// MARK: - 扫描分辨率

enum ScanResolution: String, CaseIterable, Identifiable {
    case hd720p = "720p"
    case hd1080p = "1080p"
    /*
     * 2K 是 1080p 与 4K 之间的补档。1080p 下 N=128 每格只剩 6.9 像素，
     * 4K 下 13.1 像素但采集带宽是 1080p 的四倍（发热的主要来源）。
     * 2K 约 9.8 像素，与 N=96 在 1080p 下的密度相当，带宽只有 4K 的 44%。
     *
     * AVCaptureSession 没有 2K 预设，只能直接选设备 format，因此走
     * .inputPriority；选不到合适的 format 时退回 1080p。
     */
    case qhd2K = "2K"
    /*
     * 高档位（112/128 格）在 1080p 下判决余量被压到临界点。同样的取景换到
     * 4K，每格像素翻一倍多，代价是 find_finders 的耗时按像素数线性上升。
     * 设备不支持时会自动退回 1080p。
     */
    case uhd4K = "4K"

    var id: String { rawValue }

    /// 目标横向像素数，用于挑设备 format（仅 2K 用得上）
    var targetWidth: Int {
        switch self {
        case .hd720p: return 1280
        case .hd1080p: return 1920
        case .qhd2K: return 2560
        case .uhd4K: return 3840
        }
    }
}

// MARK: - 旧 JSON 格式（兼容分支，只保留解析与重组）

struct QRChunk: Codable, Sendable {
    let fileId: String
    let fileName: String
    let totalChunks: Int
    let chunkIndex: Int
    let data: String  // base64
    let crc32: UInt32
}

/// 旧格式的单个文件状态，取代原先的 files / fileInfo 两个平行字典
final class LegacyFile: Identifiable {
    let fileId: String
    let fileName: String
    let totalChunks: Int
    var chunks: [Int: Data] = [:]

    var id: String { fileId }
    var isComplete: Bool { chunks.count == totalChunks }

    init(fileId: String, fileName: String, totalChunks: Int) {
        self.fileId = fileId
        self.fileName = fileName
        self.totalChunks = totalChunks
    }
}

struct LogEntry: Identifiable {
    let id = UUID()
    let timestamp: Date
    let message: String
    let isError: Bool
}

// MARK: - FileReceiver

@MainActor
class FileReceiver: ObservableObject {

    /// 新二进制协议的会话，以 sessionId 为主键
    @Published var sessions: [UInt32: ReceiveSession] = [:]
    /// 旧 JSON 格式的文件
    @Published var legacyFiles: [String: LegacyFile] = [:]
    @Published var logs: [LogEntry] = []
    /** 摄像头分辨率，默认 720p */
    @Published var scanResolution: ScanResolution = .hd720p
    /*
     * 诊断模式。开启后逐帧记遥测、失败帧留样落盘，用于排查；关闭时这些
     * 全部不做。留样那条路径在 4K 下一帧要拷 33 MB 再转 24 MB BMP，
     * 即便挪到后台队列也是实打实的内存与 IO 开销，常规传输不该付这个钱。
     */
    @Published var diagnosticsEnabled: Bool = false
    /*
     * 失败帧留样。单列一个开关而不是跟着诊断模式走：遥测 CSV 一帧才百来字节，
     * 留样一张 4K BMP 是 24 MB，两者的代价差着五个数量级。多数分析只要 CSV，
     * 只有需要复现某一帧的像素时才值得开这个。
     */
    @Published var saveFailureFrames: Bool = false
    /// 会话内部状态变化时递增，驱动 SwiftUI 刷新（ReceiveSession 是引用类型）
    @Published var revision: Int = 0

    /// 接收速率统计，按去重后的编码块字节数计
    let throughput = ThroughputMeter()

    /// 自动保存间隔，设计 9.3
    private let autoSaveInterval: TimeInterval = 5
    /// 进度文件写盘队列。串行，保证同一会话的先后两次写入不会乱序
    private let ioQueue = DispatchQueue(label: "vd.progress.io", qos: .utility)
    private var backgroundObserver: NSObjectProtocol?
    /// 尚未回调的消元任务数。测试用它等待异步路径跑完，见 drainForTests
    private var pendingIngests = 0

    init() {
        /*
         * Debug 构建的性能与 Release 差一个量级以上，而且差的正好是本项目最热的
         * 那类代码——紧密的 UInt64 异或循环，-Onone 下每次数组访问都带边界检查、
         * 不内联、ARC 也不消除。实测同一台机器上：
         *   1 MB 文件的回代    Debug 6026 ms / Release 26 ms
         *   秩 2000 时单块消元  Debug 9.4 ms / Release 0.03 ms
         * 这个差距已经三次被当成「性能 bug」来查（合并慢、收齐瞬间卡死、
         * 传输后半程界面卡顿），所以直接在日志里点明，别再浪费一轮排查。
         */
        #if DEBUG
        addLog("当前为 Debug 构建，喷泉码的消元与回代比 Release 慢约 300 倍；" +
               "测性能请切 Release", isError: true)
        #endif
        restoreProgress()
        backgroundObserver = NotificationCenter.default.addObserver(
            forName: UIApplication.didEnterBackgroundNotification,
            object: nil, queue: .main) { [weak self] _ in
            guard let receiver = self else { return }
            Task { @MainActor in receiver.saveAllProgress(force: true) }
        }
    }

    deinit {
        if let observer = backgroundObserver {
            NotificationCenter.default.removeObserver(observer)
        }
    }

    var sortedSessions: [ReceiveSession] {
        sessions.values.sorted { $0.sessionId < $1.sessionId }
    }

    var sortedLegacyFiles: [LegacyFile] {
        legacyFiles.values.sorted { $0.fileId < $1.fileId }
    }

    // MARK: - 日志

    func addLog(_ message: String, isError: Bool = false) {
        let formatter = DateFormatter()
        formatter.dateFormat = "HH:mm:ss"
        let timeStr = formatter.string(from: Date())
        let entry = LogEntry(timestamp: Date(), message: "[\(timeStr)] \(message)", isError: isError)
        logs.append(entry)
        if logs.count > 500 { logs.removeFirst(100) }
    }

    // MARK: - 载荷分流（设计 12.1）

    /// 按首字节分流：0x56 走新二进制协议，0x7B（'{'）走旧 JSON 分支
    @discardableResult
    func processPayload(_ payload: Data) -> Bool {
        guard let first = payload.first else { return false }
        switch first {
        case FrameParser.magic:
            return processBinaryFrame([UInt8](payload))
        case 0x7B:
            guard let text = String(data: payload, encoding: .utf8) else { return false }
            return processLegacyJSON(text)
        default:
            return false
        }
    }

    // MARK: - 新二进制协议

    private func processBinaryFrame(_ bytes: [UInt8]) -> Bool {
        guard let frame = FrameParser.parse(bytes) else {
            // 整帧丢弃：长度 / magic / 版本 / CRC 任一失败
            if bytes.count >= 6 {
                let sid = ByteOps.readUInt32(bytes, 2)
                sessions[sid]?.stats.framesRejected += 1
            }
            revision &+= 1
            return false
        }

        let session: ReceiveSession
        if let existing = sessions[frame.sessionId] {
            // 参数不一致说明 sessionId 撞车，丢弃该帧而不是污染已有会话
            guard existing.K == frame.K, existing.T == frame.T, existing.codec == frame.codec else {
                existing.stats.framesRejected += 1
                addLog("会话 \(SessionId.hex(frame.sessionId)) 参数不一致，整帧丢弃", isError: true)
                return false
            }
            session = existing
        } else {
            session = ReceiveSession(frame: frame)
            sessions[frame.sessionId] = session
            addLog("新会话 \(SessionId.hex(frame.sessionId))：K=\(frame.K) T=\(frame.T) \(frame.codec.displayName)")
        }

        // 已完成或正在后台还原的会话不再吃新块：既是白做功，也会与后台读取解码器状态打架
        guard !session.isComplete, !session.isFinalizing else {
            session.stats.framesAccepted += 1
            return false
        }

        /*
         * 消元在 ReceiveSession.workQueue 上做，这里立刻返回。
         *
         * 返回值不再表示「这一帧有没有新块」——那要等消元跑完才知道，而等待
         * 就等于把界面重新绑回解码速度。上层只用它判断「帧被接受了」，
         * 真正的进度由下面的回调推进。
         */
        pendingIngests += 1
        session.ingest(frame) { [weak self] r in
            guard let self else { return }
            self.pendingIngests -= 1
            session.apply(r)
            if r.received > r.duplicate {
                self.throughput.record(bytes: (r.received - r.duplicate) * session.T)
            }
            self.revision &+= 1

            if r.complete && !session.isFinished && !session.completionAnnounced {
                session.completionAnnounced = true
                // 收齐后不自动合并：合并是一次时长不可预测的 CPU 任务（解方程的回代是
                // O(K^2)），交给用户在主界面手动触发。但必须立刻落一次盘。
                self.persist(session)
                self.addLog("已收齐 \(session.displayName)：\(session.K) 个源块，等待手动合并")
            } else if r.gained, Date().timeIntervalSince(session.lastSavedAt) > self.autoSaveInterval {
                self.persist(session)
            }
        }
        return true
    }

    /// 完成：拼接 -> parseStream -> 解压 -> 校验 SHA-256 -> 落盘 -> 删除进度文件
    ///
    /// 全程放后台。解方程方案的回代是 O(K^2) 次块异或，K=2503 时实测 Release 约 90 ms、
    /// Debug 达 6–8 秒；放主线程会在「传输刚完成」这一刻直接冻住界面。
    private func completeSession(_ session: ReceiveSession) {
        guard !session.isFinalizing else { return }
        session.isFinalizing = true
        revision &+= 1

        session.finalizeProgress = 0
        let cancelFlag = CancellationFlag()
        session.finalizeCancel = cancelFlag
        let startedAt = Date()
        let sessionId = session.sessionId
        addLog("开始还原 \(SessionId.hex(sessionId))：K=\(session.K) T=\(session.T) \(session.codec.displayName)")

        /* 合并与收块共用同一条串行队列：两者都独占解码器，真并发起来就是数据竞争 */
        session.workQueue.async {
            var tAssemble = 0.0, tParse = 0.0, tHash = 0.0, tWrite = 0.0
            let outcome: Result<(StreamMeta, Int, URL), Error>
            do {
                var mark = Date()
                let assembled = session.assembleOnWorkQueue(
                    progress: { p in
                        Task { @MainActor in session.finalizeProgress = p }
                    },
                    isCancelled: { cancelFlag.isSet })
                tAssemble = Date().timeIntervalSince(mark)
                guard let assembled else { throw StreamError.cancelled }

                mark = Date()
                let parsed = try StreamCodec.parseStream(assembled)
                tParse = Date().timeIntervalSince(mark)

                mark = Date()
                let digestOK = StreamCodec.sha256(parsed.content) == parsed.meta.sha256
                tHash = Date().timeIntervalSince(mark)
                guard digestOK else { throw StreamError.digestMismatch }

                mark = Date()
                let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
                    .appendingPathComponent("received", isDirectory: true)
                try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
                let url = dir.appendingPathComponent(FileReceiver.sanitize(parsed.meta.fileName))
                try Data(parsed.content).write(to: url, options: .atomic)
                tWrite = Date().timeIntervalSince(mark)
                outcome = .success((parsed.meta, parsed.content.count, url))
            } catch {
                outcome = .failure(error)
            }

            Task { @MainActor [weak self] in
                session.isFinalizing = false
                session.finalizeProgress = 0
                session.finalizeCancel = nil
                let total = Int(Date().timeIntervalSince(startedAt) * 1000)
                switch outcome {
                case .success(let (meta, size, url)):
                    session.meta = meta
                    session.savedFileURL = url
                    session.finalSize = size
                    // 中间态到此为止：编码块 / 源块 / 消元基全部释放，
                    // 大文件下这部分是文件本身的 1.3–1.5 倍，留着会一直压着内存
                    session.releaseStorage()
                    ProgressStore.remove(sessionId: sessionId)
                    self?.addLog("接收完成：\(meta.fileName)（\(size) 字节，SHA-256 通过）")
                    self?.addLog(String(format: "  耗时 合计%dms = 拼接%.0f + 解流%.0f + 校验%.0f + 写盘%.0f",
                                        total, tAssemble * 1000, tParse * 1000, tHash * 1000, tWrite * 1000))
                    self?.addLog("  已释放中间态，\(Diagnostics.memoryTag())")
                case .failure(let error):
                    if case StreamError.cancelled = error {
                        session.failureMessage = nil
                        self?.addLog("会话 \(SessionId.hex(sessionId)) 合并已中止", isError: true)
                    } else {
                        session.failureMessage = error.localizedDescription
                        self?.addLog("会话 \(SessionId.hex(sessionId)) 还原失败：\(error.localizedDescription)", isError: true)
                    }
                }
                self?.revision &+= 1
            }
        }
    }

    /// 用户中止合并
    func cancelFinalize(_ session: ReceiveSession) {
        guard session.isFinalizing else { return }
        session.finalizeCancel?.set()
        addLog("正在中止 \(SessionId.hex(session.sessionId)) 的合并…")
    }

    /// 用户手动触发合并（收齐之后、中止之后、失败之后都走这里）
    func startFinalize(_ session: ReceiveSession) {
        guard !session.isFinished, !session.isFinalizing, session.isComplete else { return }
        session.failureMessage = nil
        completeSession(session)
    }

    /// 去掉路径分隔符，避免恶意文件名写到目录外
    nonisolated static func sanitize(_ name: String) -> String {
        let cleaned = name.replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "\\", with: "_")
        return cleaned.isEmpty ? "received.bin" : cleaned
    }

    // MARK: - 旧 JSON 分支

    private func processLegacyJSON(_ text: String) -> Bool {
        guard let jsonData = text.data(using: .utf8),
              let chunk = try? JSONDecoder().decode(QRChunk.self, from: jsonData) else {
            return false
        }
        let file = legacyFiles[chunk.fileId] ?? {
            let f = LegacyFile(fileId: chunk.fileId, fileName: chunk.fileName, totalChunks: chunk.totalChunks)
            legacyFiles[chunk.fileId] = f
            return f
        }()
        if file.chunks[chunk.chunkIndex] != nil { return false }

        guard let dataBytes = Data(base64Encoded: chunk.data) else {
            addLog("旧格式片段 \(chunk.chunkIndex) base64 解码失败", isError: true)
            return false
        }
        guard ByteOps.crc32([UInt8](dataBytes)) == chunk.crc32 else {
            addLog("旧格式片段 \(chunk.chunkIndex) CRC32 校验失败", isError: true)
            return false
        }
        file.chunks[chunk.chunkIndex] = dataBytes
        throughput.record(bytes: dataBytes.count)
        revision &+= 1
        addLog("旧格式片段 \(chunk.chunkIndex + 1)/\(chunk.totalChunks) - \(chunk.fileName)")
        if file.isComplete { addLog("旧格式文件 \(chunk.fileName) 接收完成") }
        return true
    }

    // MARK: - 进度持久化

    /*
     * 落盘。快照与编码必须在主 actor（解码器状态不是 Sendable），
     * 但真正的文件写入挪到串行队列——扫描时这条路径每 5 秒走一次，
     * 阻塞主 actor 会直接掉帧。串行保证同一会话的多次写入不会互相覆盖。
     */
    /*
     * 自动保存。主 actor 上只取快照，编码与写盘全部在 ioQueue 上。
     *
     * 早先编码也在主 actor 上：1 MB 文件的消元基编出来约 1.6 MB，逐字节追加
     * 在 Debug 下要几百毫秒，而这个函数每 5 秒来一次。真机表现是扫码界面持续
     * 卡顿、速率读数根本看不清，退出扫描后会话列表才一格一格补进度——那是
     * 积压的主 actor 任务在排队。取快照本身很便宜：消元基只增不改，快照里
     * 全是数组引用的持有，没有深拷贝。
     */
    private func persist(_ session: ReceiveSession) {
        let url = ProgressStore.fileURL(for: session.sessionId)
        session.lastSavedAt = Date()
        /* 快照在 workQueue 上取（解码器归它管），编码与写盘在 ioQueue 上，
           主 actor 从头到尾不参与 */
        session.snapshotForSave { [weak self] solved, pending in
            Task { @MainActor in
                guard let self else { return }
                let snap = ProgressStore.snapshot(session, solved: solved, pending: pending)
                self.ioQueue.async {
                    do {
                        let data = ProgressStore.encode([snap])
                        try ProgressStore.ensureDirectory()
                        try data.write(to: url, options: .atomic)
                    } catch {
                        let msg = error.localizedDescription
                        Task { @MainActor in self.addLog("进度保存失败：\(msg)", isError: true) }
                    }
                }
            }
        }
    }

    /*
     * 等所有在途的消元回调落地。仅供测试——真实路径不需要等，等就等于把界面
     * 重新绑回解码速度，而这次改动的全部意义就是解开这条绑定。
     */
    func drainForTests() async {
        while pendingIngests > 0 { await Task.yield() }
    }

    /// App 进入后台时全量保存
    func saveAllProgress(force: Bool) {
        let started = Date()
        var saved = 0
        // 已合并落盘的会话没有中间态可存，其进度文件在落盘时已删除；
        // 收齐但尚未合并的仍必须保存，否则退出即丢
        for session in sessions.values where !session.isFinished && !session.isReleased {
            if force || Date().timeIntervalSince(session.lastSavedAt) > autoSaveInterval {
                persist(session)
                saved += 1
            }
        }
        let ms = Diagnostics.elapsedMs(since: started)
        if force && (saved > 0 || ms > 50) {
            addLog("保存进度：\(saved) 个会话，耗时 \(ms) ms，\(Diagnostics.memoryTag())")
        }
    }

    /// App 启动时扫描目录恢复
    func restoreProgress() {
        let loaded = ProgressStore.loadAll()
        for session in loaded.sessions where sessions[session.sessionId] == nil {
            sessions[session.sessionId] = session
        }
        if !loaded.sessions.isEmpty {
            addLog("已恢复 \(loaded.sessions.count) 个会话进度")
        }
        for msg in loaded.failures {
            addLog("进度文件读取失败 \(msg)", isError: true)
        }
    }

    // MARK: - 导出/导入（跨设备迁移，与磁盘格式相同，sessionCount 可 > 1）

    func encodeProgress() throws -> Data {
        ProgressStore.encode(sortedSessions.filter { !$0.isComplete })
    }

    func loadProgress(from data: Data) throws {
        let restored = try ProgressStore.decode(data)
        for session in restored {
            sessions[session.sessionId] = session
            try? ProgressStore.save(session)
        }
        revision &+= 1
        addLog("进度已导入，包含 \(restored.count) 个会话")
    }

    // MARK: - 清理

    /// 清空全部接收数据：会话列表、旧格式文件、进度文件、已落盘的最终文件。
    ///
    /// 三处必须一起清。只删进度文件而留下内存里的 sessions，5 秒后的自动保存会把
    /// 它们原样写回，等于没清；只清 sessions 而留下 received/ 下的最终文件，
    /// 那些文件在 app 内再也没有入口可以访问到，只能占着空间。
    func clearAll() {
        sessions.removeAll()
        legacyFiles.removeAll()
        throughput.reset()
        let progressCount = ProgressStore.clearAll()
        let fileCount = FileReceiver.clearReceivedFiles()
        revision &+= 1
        addLog("数据已清空：\(progressCount) 个进度文件，\(fileCount) 个已接收文件")
    }

    /// 删除 received/ 目录下已落盘的最终文件，返回删除个数
    private static func clearReceivedFiles() -> Int {
        let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("received", isDirectory: true)
        guard let urls = try? FileManager.default.contentsOfDirectory(at: dir,
                                                                     includingPropertiesForKeys: nil) else {
            return 0
        }
        var removed = 0
        for url in urls {
            do {
                try FileManager.default.removeItem(at: url)
                removed += 1
            } catch {
                continue
            }
        }
        return removed
    }
}
