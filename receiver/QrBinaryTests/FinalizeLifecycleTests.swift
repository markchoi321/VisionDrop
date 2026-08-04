//
//  FinalizeLifecycleTests.swift
//  QrBinaryTests
//
//  完成后的生命周期：释放中间态、合并可取消、进度可观测。
//

import Testing
import Foundation
import QuartzCore
@testable import QrBinary

@MainActor
struct FinalizeLifecycleTests {

    /// 造一个已解完的会话
    private func completedSession(codec: Codec, T: Int, contentSize: Int) throws -> (ReceiveSession, [UInt8]) {
        var content = [UInt8](repeating: 0, count: contentSize)
        var s = Prng.mix32(UInt32(contentSize))
        for i in 0..<contentSize { s = Prng.xorshift32(s); content[i] = UInt8(s & 0xFF) }
        let stream = try StreamCodec.buildStream(content: content, fileName: "t.bin",
                                                 allowCompress: false).stream
        let K = (stream.count + T - 1) / T
        let composer: BlockComposer = codec == .linearSolve
            ? LinearSolveComposer(K: K) : PeelingComposer(K: K)
        let src = TestEncoder.splitSourceBlocks(stream, K: K, T: T)
        let session = ReceiveSession(sessionId: 0xABCD, K: K, T: T, codec: codec, compressed: false)
        var bid: UInt32 = 0
        while !session.withDecoderSync({ $0.isComplete }) && bid < UInt32(K * 6) {
            _ = session.withDecoderSync {
                $0.add(blockId: bid,
                       payload: TestEncoder.xorBlocks(src, composer.neighborsOf(bid), T)[...])
            }
            bid &+= 1
        }
        #expect(session.withDecoderSync({ $0.isComplete }))
        return (session, content)
    }

    @Test(arguments: [Codec.linearSolve, Codec.peeling])
    func releaseKeepsCompletionSemantics(codec: Codec) throws {
        let (session, content) = try completedSession(codec: codec, T: 293, contentSize: 40_000)
        let assembled = try #require(session.withDecoderSync {
            $0.assemble(progress: nil, isCancelled: { false }) })
        let parsed = try StreamCodec.parseStream(assembled)
        #expect(parsed.content == content)

        let K = session.K
        session.withDecoderSync { $0.releaseStorage() }

        // 释放后仍要表现为「已完成」，否则界面会把它当成半截会话重新开始收
        #expect(session.withDecoderSync({ $0.isReleased }))
        #expect(session.withDecoderSync({ $0.isComplete }))
        #expect(session.isComplete)
        #expect(session.withDecoderSync({ $0.solvedCount }) == K)
        #expect(session.withDecoderSync({ $0.pendingCount }) == 0)
        // 中间态必须真的没了
        #expect(session.withDecoderSync({ $0.snapshotSolved() }).isEmpty)
        #expect(session.withDecoderSync({ $0.snapshotPending() }).isEmpty)
        if let peeling = session.withDecoderSync({ $0 as? PeelingDecoder }) {
            #expect(peeling.solvedBlock(0) == nil, "释放后读源块不得越界")
        }
    }

    /// 释放后 tryExtractMeta 不得崩溃（它会去读源块）
    @Test func metaExtractionSafeAfterRelease() throws {
        let (session, _) = try completedSession(codec: .peeling, T: 293, contentSize: 40_000)
        session.withDecoderSync { $0.releaseStorage() }
        session.meta = nil
        session.tryExtractMeta()      // 不崩即可
        #expect(session.meta == nil)
    }

    /// 合并可以中途取消
    @Test func assembleCanBeCancelled() throws {
        let (session, _) = try completedSession(codec: .linearSolve, T: 419, contentSize: 200_000)
        let flag = CancellationFlag()
        flag.set()
        let out = session.withDecoderSync { $0.assemble(progress: nil, isCancelled: { flag.isSet }) }
        #expect(out == nil, "已置位取消标志时必须返回 nil")
    }

    /// 取消标志跨线程可见
    @Test func cancellationFlagIsThreadSafe() async {
        let flag = CancellationFlag()
        #expect(!flag.isSet)
        await Task.detached { flag.set() }.value
        #expect(flag.isSet)
    }

    /// 进度回调必须推进到 1.0
    @Test(arguments: [Codec.linearSolve, Codec.peeling])
    func assembleReportsProgress(codec: Codec) throws {
        let (session, _) = try completedSession(codec: codec, T: 293, contentSize: 60_000)
        final class Box: @unchecked Sendable { var last = -1.0; var count = 0 }
        let box = Box()
        let out = session.withDecoderSync { d in
            d.assemble(progress: { p in
                box.last = p
                box.count += 1
            }, isCancelled: { false })
        }
        #expect(out != nil)
        #expect(box.count > 0, "至少要回调一次")
        #expect(box.last == 1.0, "结束时进度必须是 1.0")
    }
}

// MARK: - 手动合并流程

@MainActor
struct ManualFinalizeTests {

    /// 收齐后不得自动合并：合并耗时不可预测，必须由用户触发
    @Test func completionDoesNotAutoMerge() throws {
        let vectors = try Vectors.binary("stream.bin")
        let e2e = try Vectors.keyValues("e2e.txt")
        let T = Int(e2e["rlnc.T"]!)!, K = Int(e2e["rlnc.K"]!)!
        let sid = UInt32(e2e["rlnc.sessionId"]!, radix: 16)!
        let src = TestEncoder.splitSourceBlocks(vectors, K: K, T: T)
        let composer = LinearSolveComposer(K: K)

        let receiver = FileReceiver()
        receiver.clearAll()
        var base: UInt32 = 0
        let m = 40
        while receiver.sessions[sid]?.isComplete != true && base < UInt32(K * 4) {
            let bytes = TestEncoder.encodeFrame(src: src, K: K, T: T, codec: .linearSolve,
                                                compressed: false, sessionId: sid,
                                                baseBlockId: base, m: m, composer: composer)
            _ = receiver.processPayload(Data(bytes))
            base &+= UInt32(m)
        }
        let session = try #require(receiver.sessions[sid])
        #expect(session.isComplete, "块应已收齐")
        #expect(!session.isFinalizing, "不得自动开始合并")
        #expect(!session.isFinished, "不得自动落盘")
        #expect(!session.withDecoderSync({ $0.isReleased }), "未合并前中间态必须保留")
    }

    /// 收齐但未合并的会话必须能被保存，否则退出即丢
    @Test func collectedSessionIsPersistable() throws {
        let stream = try Vectors.binary("stream.bin")
        let e2e = try Vectors.keyValues("e2e.txt")
        let T = Int(e2e["lt.T"]!)!, K = Int(e2e["lt.K"]!)!
        let src = TestEncoder.splitSourceBlocks(stream, K: K, T: T)
        let composer = PeelingComposer(K: K)
        let session = ReceiveSession(sessionId: 0x5150, K: K, T: T, codec: .peeling, compressed: false)
        var bid: UInt32 = 0
        while !session.withDecoderSync({ $0.isComplete }) && bid < UInt32(K * 6) {
            _ = session.withDecoderSync {
                $0.add(blockId: bid,
                       payload: TestEncoder.xorBlocks(src, composer.neighborsOf(bid), T)[...])
            }
            bid &+= 1
        }
        #expect(session.isComplete && !session.isFinished)
        // 快照必须仍然拿得到全部源块
        #expect(session.withDecoderSync({ $0.snapshotSolved() }).count == K)
        let blob = ProgressStore.encode([session])
        #expect(blob.count > K * T, "序列化必须包含全部源块")
    }
}

// MARK: - 清理

/*
 * 串行执行。这一组用例共用 Documents/progress 与 received 两个目录，clearAll
 * 会把它们整个删掉；并行跑时一个用例的清理会踩到另一个用例正在写的进度文件。
 * 消元改成异步之后这个窗口被拉长，失败从偶发变成常态——根因一直在，不是新引入的。
 */
@Suite(.serialized)
@MainActor
struct ClearAllTests {

    private var receivedDir: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("received", isDirectory: true)
    }

    /// 造一个进行中的会话，同时留下进度文件。
    ///
    /// 先 clearAll 是必须的：本方法自己会把进度存盘，而本组用例并行跑、
    /// 共用同一个 Documents 目录，新建的 FileReceiver 会把上一个用例存的
    /// 同 sessionId 进度读回来。那样种子帧就成了重复块被丢弃，
    /// 既不计吞吐也不进会话表，表现为随机失败。
    private func seedSession(_ receiver: FileReceiver) throws -> UInt32 {
        receiver.clearAll()
        let stream = try Vectors.binary("stream.bin")
        let e2e = try Vectors.keyValues("e2e.txt")
        let T = Int(e2e["rlnc.T"]!)!, K = Int(e2e["rlnc.K"]!)!
        let sid = UInt32(e2e["rlnc.sessionId"]!, radix: 16)!
        let src = TestEncoder.splitSourceBlocks(stream, K: K, T: T)
        let bytes = TestEncoder.encodeFrame(src: src, K: K, T: T, codec: .linearSolve,
                                            compressed: false, sessionId: sid,
                                            baseBlockId: 0, m: 8,
                                            composer: LinearSolveComposer(K: K))
        _ = receiver.processPayload(Data(bytes))
        try ProgressStore.save(#require(receiver.sessions[sid]))
        return sid
    }

    /// 清理必须真的清掉会话列表。
    /// 早前按钮接的是只删磁盘进度文件的入口，内存里的 sessions 一个没动，
    /// 界面看不出任何变化，而且 5 秒后的自动保存会把进度文件原样写回。
    @Test func clearAllEmptiesSessionList() throws {
        let receiver = FileReceiver()
        let sid = try seedSession(receiver)
        #expect(receiver.sessions[sid] != nil)
        #expect(!receiver.sortedSessions.isEmpty)

        receiver.clearAll()

        #expect(receiver.sessions.isEmpty, "会话列表必须被清空")
        #expect(receiver.sortedSessions.isEmpty)
        #expect(receiver.legacyFiles.isEmpty)
    }

    /// 清理后重启不得把会话恢复回来：磁盘进度文件也必须一并删除
    @Test func clearAllRemovesProgressFiles() throws {
        let receiver = FileReceiver()
        _ = try seedSession(receiver)
        receiver.clearAll()

        // 新实例走 restoreProgress，若进度文件还在就会把会话读回来
        let reopened = FileReceiver()
        #expect(reopened.sessions.isEmpty, "清理后重启不得恢复出会话")
    }

    /// 已落盘的最终文件也要删：会话记录没了之后，app 内再没有入口能访问到它们
    @Test func clearAllRemovesReceivedFiles() throws {
        let fm = FileManager.default
        try fm.createDirectory(at: receivedDir, withIntermediateDirectories: true)
        let marker = receivedDir.appendingPathComponent("clear-test.bin")
        try Data([1, 2, 3]).write(to: marker)
        #expect(fm.fileExists(atPath: marker.path))

        let receiver = FileReceiver()
        receiver.clearAll()

        #expect(!fm.fileExists(atPath: marker.path), "已接收文件必须一并删除")
    }

    /// 速率读数也要清零，否则清空后界面还挂着上次的速率。
    ///
    /// 读数一律传入 seed 之前的时刻：计量窗口只有 1 秒，而测试是并行跑的，
    /// 机器负载高时 seedSession 到断言之间真能超过 1 秒，样本被窗口淘汰、
    /// 首个断言假失败。这里要验的是 clearAll 的行为，不是墙钟。
    @MainActor
    @Test func clearAllResetsThroughput() async throws {
        let receiver = FileReceiver()
        let t0 = CACurrentMediaTime()
        _ = try seedSession(receiver)
        await receiver.drainForTests()
        #expect(receiver.throughput.bytesPerSecond(now: t0) > 0)
        receiver.clearAll()
        #expect(receiver.throughput.bytesPerSecond(now: t0) == 0)
    }
}
