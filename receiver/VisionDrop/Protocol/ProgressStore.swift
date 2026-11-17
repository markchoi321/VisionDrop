//
//  ProgressStore.swift
//  VisionDrop
//
//  进度持久化。严格对应 CONTRACT.md 第 11 节的二进制格式（全大端，不用 JSON）。
//  位置：Documents/progress/<sessionId 8位小写hex>.vdpg，每会话一个文件。
//

import Foundation

enum ProgressStoreError: Error, LocalizedError {
    case badMagic
    case unsupportedVersion(Int)
    case truncated

    var errorDescription: String? {
        switch self {
        case .badMagic: return "进度文件 magic 不匹配"
        case .unsupportedVersion(let v): return "进度文件版本不支持: \(v)"
        case .truncated: return "进度文件被截断"
        }
    }
}

enum ProgressStore {

    static let magic: [UInt8] = [0x56, 0x44, 0x50, 0x47]     // "VDPG"
    /*
     * 版本 2：未消元记录改为按稀疏度分别编码（见 PendingRecord）。
     * RLNC 的稠密系数向量原来逐个置位写 3 字节索引，K=2517 时整个基约 10MB，
     * 自动存盘要 300ms+；改用位图后同样的基约 1MB。
     * 版本 1 的文件不再读取——进度文件是可再生的临时状态，
     * 加迁移代码的价值低于把格式保持简单。
     */
    static let formatVersion: UInt16 = 2
    static let statsLength = 40

    // MARK: - 目录

    static var directory: URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        return docs.appendingPathComponent("progress", isDirectory: true)
    }

    static func fileURL(for sessionId: UInt32) -> URL {
        directory.appendingPathComponent("\(SessionId.hex(sessionId)).vdpg")
    }

    static func ensureDirectory() throws {
        let dir = directory
        if !FileManager.default.fileExists(atPath: dir.path) {
            try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        }
    }

    // MARK: - 编码

    /*
     * 一个会话的可搬运快照。
     *
     * 取快照必须在主 actor 上做（要读解码器状态），但编码是纯数据变换，
     * 可以拿到后台去——这是必须拆开的理由：1 MB 文件的消元基编码出来约 1.6 MB，
     * 逐字节追加在 Debug 下要几百毫秒，而自动保存每 5 秒来一次。真机表现是
     * 扫码界面持续卡顿、速率读数根本看不清，退出扫描后会话列表才一格一格
     * 补上进度——那是积压的主 actor 任务在排队。
     *
     * 快照本身很便宜：消元基是只增不改的（add 只在空槽位写入，已有的行永远
     * 不动），所以这里全是数组引用的持有，没有深拷贝。
     */
    struct SessionSnapshot: Sendable {
        var sessionId: UInt32
        var K: Int
        var T: Int
        var flags: UInt8
        var metaBytes: [UInt8]
        var solved: [SolvedRecord]
        var pending: [PendingRecord]
        var framesAccepted: Int
        var framesRejected: Int
        var blocksReceived: Int
        var blocksDuplicate: Int
        var maxBlockId: UInt32
        var firstSeenAt: Double
        var lastSeenAt: Double
    }

    /*
     * 组装快照。解码器部分由调用方在 workQueue 上取好传进来——解码器的归属
     * 在那条队列上，主 actor 不许直接碰。
     */
    static func snapshot(_ s: ReceiveSession,
                         solved: [SolvedRecord],
                         pending: [PendingRecord]) -> SessionSnapshot {
        var flags: UInt8 = 0
        if s.compressed { flags |= 0x01 }
        if s.meta != nil { flags |= 0x02 }
        if s.codec == .peeling { flags |= 0x04 }
        return SessionSnapshot(
            sessionId: s.sessionId, K: s.K, T: s.T, flags: flags,
            metaBytes: s.meta?.rawHeader ?? [],
            solved: solved,
            pending: pending,
            framesAccepted: s.stats.framesAccepted,
            framesRejected: s.stats.framesRejected,
            blocksReceived: s.stats.blocksReceived,
            blocksDuplicate: s.stats.blocksDuplicate,
            maxBlockId: s.stats.maxBlockId,
            firstSeenAt: s.stats.firstSeenAt.timeIntervalSince1970,
            lastSeenAt: s.stats.lastSeenAt.timeIntervalSince1970)
    }

    /// 打包多个会话，sessionCount 可 > 1，用于跨设备迁移导出
    /// 便利重载：同步取快照后编码。会短暂占用 workQueue，仅用于测试与导出。
    static func encode(_ sessions: [ReceiveSession]) -> Data {
        encode(sessions.map { s in
            s.withDecoderSync { d in
                snapshot(s, solved: d.snapshotSolved(), pending: d.snapshotPending())
            }
        })
    }

    /// 快照 -> 字节。纯数据变换，可在任意线程执行
    static func encode(_ snapshots: [SessionSnapshot]) -> Data {
        var out: [UInt8] = []
        out.append(contentsOf: magic)
        ByteOps.appendUInt16(&out, formatVersion)
        ByteOps.appendUInt16(&out, UInt16(min(snapshots.count, Int(UInt16.max))))
        for s in snapshots { appendSession(&out, s) }
        return Data(out)
    }

    private static func appendSession(_ out: inout [UInt8], _ s: SessionSnapshot) {
        ByteOps.appendUInt32(&out, s.sessionId)
        ByteOps.appendUInt24(&out, UInt32(s.K))
        ByteOps.appendUInt16(&out, UInt16(s.T))
        out.append(s.flags)

        ByteOps.appendUInt16(&out, UInt16(s.metaBytes.count))
        out.append(contentsOf: s.metaBytes)

        let solved = s.solved
        ByteOps.appendUInt32(&out, UInt32(solved.count))
        for rec in solved {
            ByteOps.appendUInt24(&out, UInt32(rec.index))
            out.append(contentsOf: rec.data)
        }

        let pending = s.pending
        ByteOps.appendUInt32(&out, UInt32(pending.count))
        for rec in pending {
            ByteOps.appendUInt32(&out, rec.blockId)
            if let bits = rec.coeff {
                out.append(1)                                   // 1 = 稠密位图
                ByteOps.appendUInt24(&out, UInt32(bits.bitCount))
                for w in bits.words { ByteOps.appendUInt64(&out, w) }
            } else {
                out.append(0)                                   // 0 = 稀疏索引表
                ByteOps.appendUInt16(&out, UInt16(rec.neighbors.count))
                for i in rec.neighbors { ByteOps.appendUInt24(&out, UInt32(i)) }
            }
            out.append(contentsOf: rec.payload)
        }

        ByteOps.appendUInt32(&out, UInt32(s.framesAccepted))
        ByteOps.appendUInt32(&out, UInt32(s.framesRejected))
        ByteOps.appendUInt32(&out, UInt32(s.blocksReceived))
        ByteOps.appendUInt32(&out, UInt32(s.blocksDuplicate))
        ByteOps.appendUInt32(&out, s.maxBlockId)
        ByteOps.appendUInt32(&out, 0)                                   // reserved
        ByteOps.appendDouble(&out, s.firstSeenAt)
        ByteOps.appendDouble(&out, s.lastSeenAt)
    }

    // MARK: - 解码

    static func decode(_ data: Data) throws -> [ReceiveSession] {
        let b = [UInt8](data)
        guard b.count >= 8 else { throw ProgressStoreError.truncated }
        guard Array(b[0..<4]) == magic else { throw ProgressStoreError.badMagic }
        let version = ByteOps.readUInt16(b, 4)
        guard version == formatVersion else { throw ProgressStoreError.unsupportedVersion(Int(version)) }
        let count = Int(ByteOps.readUInt16(b, 6))

        var off = 8
        var out: [ReceiveSession] = []
        for _ in 0..<count {
            let (session, next) = try decodeSession(b, off)
            out.append(session)
            off = next
        }
        return out
    }

    private static func decodeSession(_ b: [UInt8], _ start: Int) throws -> (ReceiveSession, Int) {
        var off = start
        func need(_ n: Int) throws {
            guard off + n <= b.count else { throw ProgressStoreError.truncated }
        }

        try need(12)
        let sessionId = ByteOps.readUInt32(b, off); off += 4
        let K = Int(ByteOps.readUInt24(b, off)); off += 3
        let T = Int(ByteOps.readUInt16(b, off)); off += 2
        let flags = b[off]; off += 1
        let metaLen = Int(ByteOps.readUInt16(b, off)); off += 2

        guard K > 0, T > 0 else { throw ProgressStoreError.truncated }
        let compressed = flags & 0x01 != 0
        let codec: Codec = (flags & 0x04 != 0) ? .peeling : .linearSolve

        try need(metaLen)
        var meta: StreamMeta?
        if metaLen > 0 {
            if let m = try? StreamCodec.parseHeader(Array(b[off..<(off + metaLen)])) { meta = m }
            off += metaLen
        }

        try need(4)
        let solvedCount = Int(ByteOps.readUInt32(b, off)); off += 4
        var solved: [SolvedRecord] = []
        solved.reserveCapacity(solvedCount)
        for _ in 0..<solvedCount {
            try need(3 + T)
            let idx = Int(ByteOps.readUInt24(b, off)); off += 3
            solved.append(SolvedRecord(index: idx, data: Array(b[off..<(off + T)])))
            off += T
        }

        try need(4)
        let pendingCount = Int(ByteOps.readUInt32(b, off)); off += 4
        var pending: [PendingRecord] = []
        pending.reserveCapacity(pendingCount)
        for _ in 0..<pendingCount {
            try need(5)
            let blockId = ByteOps.readUInt32(b, off); off += 4
            let kind = b[off]; off += 1
            if kind == 1 {
                try need(3)
                let bitCount = Int(ByteOps.readUInt24(b, off)); off += 3
                let wordCount = (bitCount + 63) / 64
                try need(wordCount * 8 + T)
                var words: [UInt64] = []
                words.reserveCapacity(wordCount)
                for _ in 0..<wordCount {
                    words.append(ByteOps.readUInt64(b, off)); off += 8
                }
                pending.append(PendingRecord(
                    blockId: blockId,
                    coeff: BitSet(bitCount: bitCount, words64: words),
                    payload: Array(b[off..<(off + T)])))
            } else {
                try need(2)
                let degree = Int(ByteOps.readUInt16(b, off)); off += 2
                try need(degree * 3 + T)
                var neighbors: [Int] = []
                neighbors.reserveCapacity(degree)
                for _ in 0..<degree {
                    neighbors.append(Int(ByteOps.readUInt24(b, off))); off += 3
                }
                pending.append(PendingRecord(blockId: blockId, neighbors: neighbors,
                                             payload: Array(b[off..<(off + T)])))
            }
            off += T
        }

        try need(statsLength)
        var stats = SessionStats()
        stats.framesAccepted = Int(ByteOps.readUInt32(b, off))
        stats.framesRejected = Int(ByteOps.readUInt32(b, off + 4))
        stats.blocksReceived = Int(ByteOps.readUInt32(b, off + 8))
        stats.blocksDuplicate = Int(ByteOps.readUInt32(b, off + 12))
        stats.maxBlockId = ByteOps.readUInt32(b, off + 16)
        stats.firstSeenAt = Date(timeIntervalSince1970: ByteOps.readDouble(b, off + 24))
        stats.lastSeenAt = Date(timeIntervalSince1970: ByteOps.readDouble(b, off + 32))
        off += statsLength

        let session = ReceiveSession(sessionId: sessionId, K: K, T: T,
                                     codec: codec, compressed: compressed, stats: stats)
        session.meta = meta
        session.restoreProgress(solved: solved, pending: pending)
        session.lastSavedAt = Date()
        return (session, off)
    }

    // MARK: - 磁盘读写

    static func save(_ session: ReceiveSession) throws {
        try ensureDirectory()
        let data = encode([session])
        try data.write(to: fileURL(for: session.sessionId), options: .atomic)
        session.lastSavedAt = Date()
    }

    /// App 启动时扫描目录恢复全部会话
    static func loadAll() -> (sessions: [ReceiveSession], failures: [String]) {
        var sessions: [ReceiveSession] = []
        var failures: [String] = []
        let fm = FileManager.default
        guard let items = try? fm.contentsOfDirectory(at: directory,
                                                      includingPropertiesForKeys: nil) else {
            return ([], [])
        }
        for url in items where url.pathExtension == "vdpg" {
            do {
                let data = try Data(contentsOf: url)
                sessions.append(contentsOf: try decode(data))
            } catch {
                failures.append("\(url.lastPathComponent): \(error.localizedDescription)")
            }
        }
        return (sessions, failures)
    }

    static func remove(sessionId: UInt32) {
        try? FileManager.default.removeItem(at: fileURL(for: sessionId))
    }

    /// 手动清理入口：删除全部进度文件
    @discardableResult
    static func clearAll() -> Int {
        let fm = FileManager.default
        guard let items = try? fm.contentsOfDirectory(at: directory, includingPropertiesForKeys: nil) else {
            return 0
        }
        var n = 0
        for url in items where url.pathExtension == "vdpg" {
            if (try? fm.removeItem(at: url)) != nil { n += 1 }
        }
        return n
    }
}
