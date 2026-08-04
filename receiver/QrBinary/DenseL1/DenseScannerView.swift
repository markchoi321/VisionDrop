//
//  DenseScannerView.swift
//  QrBinary
//
//  VD-D1 密集彩色模式的扫描界面。与二维码扫描界面分开，理由同发送端把
//  dense.html 独立成页：两条物理层的取景要求与可调项不同，合并只会互相牵制。
//
//  解出的字节走同一套 FileReceiver.processPayload，L2 之上完全共用。
//

import AVFoundation
import Combine
import SwiftUI

struct DenseScannerView: View {
    @ObservedObject var receiver: FileReceiver
    @Environment(\.dismiss) private var dismiss

    @StateObject private var scanner = DenseCameraScanner()
    @State private var grid: Int32 = DenseDecoder.gridAuto
    @State private var lastInfo = ""
    @State private var detectedFps = 0.0
    @State private var processedFps = 0.0
    @State private var rawBps = 0.0
    @State private var netBps = 0.0

    /*
     * 读数刷新频率。计量窗口本身固定 1 秒（见 ThroughputMeter），这里只决定
     * 多久取一次数。原来 1 秒取一次，读数最长滞后整整一秒，相机忙时定时器
     * 还会被推迟，看上去就像没在统计最近 1 秒。改成 5Hz：窗口仍是 1 秒，
     * 但每 200ms 重算一次，数字跟得上取景动作。
     */
    private let rateTimer = Timer.publish(every: 0.2, on: .main, in: .common).autoconnect()

    /// 0 = 自动判别。符号不携带 N，手工对齐两端档位极易出错，
    /// 因此默认自动；固定档位仅用于排查。
    private static let grids: [Int32] = [DenseDecoder.gridAuto, 64, 80, 96, 112, 128]

    var body: some View {
        NavigationStack {
            ZStack(alignment: .bottom) {
                DenseCameraPreview(session: scanner.session)
                    .ignoresSafeArea()

                VStack(spacing: 8) {
                    Picker("档位", selection: $grid) {
                        ForEach(Self.grids, id: \.self) { n in
                            Text(n == DenseDecoder.gridAuto ? "自动" : "\(n)").tag(n)
                        }
                    }
                    .pickerStyle(.segmented)
                    .onChange(of: grid) { _, newValue in
                        scanner.grid = newValue
                        scanner.resetTracking()
                    }

                    /*
                     * 指标区。四个读数两两成对，标签必须能一眼分清：
                     * 「识别帧率」是解码器的产出，「送检帧率」是相机的输入，
                     * 两者的差就是解不出来的那部分；
                     * 「原始速率」含喷泉码冗余与重复帧，「有效速率」是去重后
                     * 真正推进传输的量，两者的差就是重复扫到的代价。
                     */
                    HStack(spacing: 16) {
                        metric("识别帧率", String(format: "%.1f", detectedFps), "fps")
                        metric("送检帧率", String(format: "%.1f", processedFps), "fps")
                    }
                    HStack(spacing: 16) {
                        metric("原始速率", ThroughputMeter.format(rawBps), "")
                        metric("有效速率", ThroughputMeter.format(netBps), "")
                    }
                    Label(scanner.isLocked ? "曝光已锁定" : "曝光自动",
                          systemImage: scanner.isLocked ? "lock.fill" : "lock.open")
                        .font(.caption)

                    /*
                     * 下面三行**始终占位**，内容为空时也保留行高。
                     * 早先是条件渲染，一帧解出、下一帧失败就会让整栏高矮跳动，
                     * 取景时画面跟着上下弹，既晃眼又容易误触。行数固定后，
                     * 文字变化不再改变布局。
                     */
                    Text(scanner.lastFailure)
                        .font(.caption)
                        .foregroundStyle(.orange)
                        .lineLimit(1, reservesSpace: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    // 现场标定读数：档位、每格像素、镜头弓曲、头部副本分歧
                    Text(scanner.diagText)
                        .font(.caption2.monospacedDigit())
                        .foregroundStyle(.secondary)
                        .lineLimit(1, reservesSpace: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    Text(lastInfo)
                        .font(.caption.monospacedDigit())
                        .lineLimit(1, reservesSpace: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
                .padding()
                .background(.ultraThinMaterial)
            }
            .navigationTitle("密集 8 色扫描")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("完成") {
                        dismiss()
                        Task { @MainActor in receiver.saveAllProgress(force: true) }
                    }
                }
                ToolbarItem(placement: .primaryAction) {
                    Menu {
                        Button("重新取景") { scanner.resetTracking() }
                        // 现场诊断：导出的 BMP 可直接喂给 C 侧 harness 复现问题
                        Button("导出诊断帧") { scanner.dumpNextFrame() }
                        /* 诊断包 = 逐帧遥测 CSV + 最多 10 张失败帧原图。
                           CSV 定位卡在哪一环，原图用来复现 */
                        Button("导出诊断包（\(scanner.logCount) 条 / \(scanner.failureShotCount) 张）") {
                            scanner.exportTelemetry()
                        }
                        .disabled(!receiver.diagnosticsEnabled)
                    } label: {
                        Image(systemName: "ellipsis.circle")
                    }
                }
            }
            .onAppear {
                scanner.grid = grid
                scanner.resolution = receiver.scanResolution
                scanner.diagnosticsEnabled = receiver.diagnosticsEnabled
                scanner.onFrameDecoded = { [weak receiver] frame in
                    guard let receiver = receiver else { return }
                    Task { @MainActor in
                        guard receiver.processPayload(frame) else { return }
                        if let s = receiver.sortedSessions.last(where: { !$0.isComplete }) {
                            lastInfo = "\(s.displayName)  \(s.stats.blocksUnique)/\(s.estimatedNeededBlocks) 块"
                        } else {
                            lastInfo = ""
                        }
                    }
                }
                scanner.start()
            }
            .onDisappear {
                scanner.stop()
                Task { @MainActor in receiver.saveAllProgress(force: true) }
            }
            .onReceive(rateTimer) { _ in
                detectedFps = scanner.detectedMeter.perSecond()
                processedFps = scanner.processedMeter.perSecond()
                rawBps = scanner.rawByteMeter.bytesPerSecond()
                netBps = scanner.netByteMeter.bytesPerSecond()
            }
            .onChange(of: scanner.dumpedFrameURL) { _, url in
                guard let url else { return }
                FileExport.share(url: url)
            }
            .onChange(of: scanner.exportedLogURL) { _, url in
                guard let url else { return }
                FileExport.share(url: url)
            }
        }
    }
}

/// 一个指标读数：小字标签 + 等宽数字，保证两两并排时对得齐
@ViewBuilder
private func metric(_ title: String, _ value: String, _ unit: String) -> some View {
    VStack(spacing: 1) {
        Text(title).font(.caption2).foregroundStyle(.secondary)
        Text(unit.isEmpty ? value : "\(value) \(unit)")
            .font(.caption.monospacedDigit().weight(.semibold))
    }
}

/// 相机预览层。与 QR 路径的 CameraPreview 分开，避免跨模式共享可变状态。
struct DenseCameraPreview: UIViewRepresentable {
    let session: AVCaptureSession

    func makeUIView(context: Context) -> PreviewView {
        let v = PreviewView()
        v.previewLayer.session = session
        v.previewLayer.videoGravity = .resizeAspect
        return v
    }

    func updateUIView(_ uiView: PreviewView, context: Context) {}

    final class PreviewView: UIView {
        override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
        var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
    }
}
