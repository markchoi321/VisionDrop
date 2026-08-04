# VisionDrop

通过屏幕与摄像头之间的单向光学信道传输文件：发送端把文件编码成滚动播放的密集 8 色色块码序列，接收端用摄像头连续扫描并重组出原始二进制文件。全程不依赖网络、蓝牙、配对或可移动介质，适用于物理隔离环境。

## 仓库结构

```
VisionDrop/
├── protocol/            协议契约、参考实现与跨语言测试向量
│   ├── CONTRACT.md      两端实现的唯一权威
│   ├── refimpl.py       Python 参考实现（编解码 + 选参 + 档位）
│   ├── crosscheck.py    跨语言校验：真实帧字节 -> 参考解码器 -> 比对哈希
│   ├── vectors/         L2–L5 测试向量，两端各自比对
│   └── dense/           密集 8 色 L1（VD-D1）：规约 + 参考实现 + 向量 + 金图像
├── sender/              发送端
│   ├── java/            L2–L5 协议层交叉校验模块（不是可运行的发送端）
│   └── web/             纯原生 HTML+JS 版（零依赖，兼容 IE10+ / Android 4+）
├── receiver/            接收端（iOS，SwiftUI）
└── README.md
```

## 协议

五层结构，详见 [protocol/CONTRACT.md](protocol/CONTRACT.md)：

| 层 | 职责 |
|---|---|
| L5 会话 | 多会话并发、进度持久化、确定性 sessionId 续传 |
| L4 流 | 元数据头 + 可选 raw deflate + 文件内容 |
| L3 编码 | 喷泉码，按文件大小分档选用解方程（RLNC）或剥洋葱（LT） |
| L2 帧 | 20 字节二进制帧头 + m 个无头编码块 |
| L1 物理 | 密集 8 色色块码（VD-D1）生成/识别，每格 3 bit，档位 N=64–128 |

三个要点：

- **无 JSON 无 Base64**。20 字节帧头承载 sessionId / K / T / m / baseBlockId / CRC32，文件名等元信息内嵌进流层，不再每帧重复。单帧有效载荷率从 68.9% 提升到 99.2%（V40）。
- **喷泉码**。发送端持续发出编码块，接收端收够任意数量即可还原，不需要指定重发哪一块——手工补帧与重播轮次的概念都消失了。丢帧代价严格线性：50% 丢帧只是帧数翻倍，不存在长尾。
- **块与帧解耦**。源块大小 T 与每帧块数 m 相互独立：帧头显式携带 m，接收端按帧头解析，L1 的档位选择不进入 L2–L5 语义。只要 `T`/`K`/`codec`/`sessionId` 不变，**接收端已收到的编码块继续有效**。（色块码为提升填充率给每档各自选 T，因此换档会换会话，详见 [sender/web/README.md](sender/web/README.md)。）

编码块只在帧头携带 `baseBlockId`，参与异或的源块由两端用同一 PRNG 从块序号推导，不占载荷。这要求两端 PRNG 与度分布逐位一致，故有 `protocol/vectors/`。

## 两端现状

### 发送端 sender/web

纯原生 HTML + JS，零依赖零构建，双击 `dense.html` 即可运行。协议层与密集
8 色 L1 编码器全部 ES5 实现，兼容 IE10+ / Android 4+ / iOS 6+。与桌面版的
唯一语义差异是不做流层压缩（flag 恒为 0，契约允许）。播放游标存
localStorage，同一文件同参数重传自动续接。

物理层为 VD-D1：借鉴 JAB Code 的 RGB 立方体调色板，每格 3 bit，档位
N=64–128，N=128 档单符号净载荷 5087 字节。每档各自选块大小 T 以吃满版面。
规约见 [protocol/dense/DENSE.md](protocol/dense/DENSE.md)。

详见 [sender/web/README.md](sender/web/README.md)。

### 协议层交叉校验 sender/java

**不是可运行的发送端。** 二维码物理层迁出后，这里只保留一份独立的 L2–L5 协议实现与
向量比对（Swing 界面、播放状态持久化与 ZXing 依赖已删除）。留着的理由是
`protocol/vectors/` 需要多个独立实现互验：L2–L5 与物理层无关，色块码路径原样沿用同一套
语义，少一个独立实现就少一层保障，而成本只是一个不再运行的 Maven 模块。

详见 [sender/java/README.md](sender/java/README.md)。

### 接收端 receiver

SwiftUI 的 iOS 应用。摄像头连续扫描，多会话并行维护，进度以二进制 `.vdpg` 格式自动落盘并在启动时恢复。

识别管线只有一条（`QrBinary/DenseL1/`），解码核心为零依赖的纯 C99：亮度域游程定位、几何校正、逐格面积平均采样、定标判决、RS errata 纠错，跟踪态 N=96 单帧 0.5ms（30fps 预算 33ms）。相机侧用 BGRA 输出并在稳定后锁定 AE/AWB/对焦——多电平判决最怕的就是相机自动调节带来的光度漂移。该管线不依赖 Vision，**模拟器可跑全流程回归**。详见 [receiver/QrBinary/DenseL1/README.md](receiver/QrBinary/DenseL1/README.md)。

扫描帧率上限 60fps。超过 30fps 需显式选择支持高帧率的 `activeFormat`（`sessionPreset` 隐含 30fps 上限）。这直接放宽了发送端的显示间隔下限：按 `N = 间隔 × 帧率 / 1000 ≥ 2.5`，30fps 对应 83ms，60fps 对应 42ms。

接收端 README 记录了扫描帧率与发送端显示间隔之间的**采样混叠模型**——每个符号的期望扫描次数 `N = 显示间隔 × 帧率 / 1000`，经验法则是 `N ≥ 2.5`，且应避开整数比以防相位锁定。这套模型是所有参数调整的依据。

详见 [receiver/README.md](receiver/README.md)。

## 验证

```bash
python3 protocol/refimpl.py check                      # 参考实现自检
mvn -o -f sender/java/pom.xml test                     # Java 协议层交叉校验
node sender/web/test/run-vectors.js                    # Web 发送端 L2–L5 向量比对
node sender/web/test/run-dense-vectors.js              # Web 发送端 L1（VD-D1）向量比对
python3 protocol/dense/dense_refimpl.py check          # 密集 8 色参考实现自检
xcodebuild test -project receiver/QrBinary.xcodeproj \
  -scheme QrBinary -destination 'platform=iOS Simulator,name=iPhone 17'
```

两端各自比对 `protocol/vectors/` 下的全部向量（PRNG、量化度分布、邻居集合、系数向量、选参、档位、整帧字节、raw deflate 往返），并各自跑 0/10/30/50% 丢帧的端到端还原与 SHA-256 校验。

跨语言联测：发送端导出帧转储后

```bash
python3 protocol/crosscheck.py <dump> --sha <原始文件sha256>
```

**注意**：Swift 解码器在 Debug 构建下比 Release 慢约 50 倍。真机现场标定务必用 Release 构建，否则大文件的实时消元跟不上扫描速度。

## 待办

- 识别率标定（设计文档 11.1）：档位扫描 + 显示尺寸，决定 720p / 1080p 各自的可用档位上限
- 密集 8 色模式：真机信道标定；给格式补内部校正图形（替掉现在从边缘外推畸变的权宜做法）；档位 N 写进符号头部。解码核心与相机管线已完成，见 [receiver/QrBinary/DenseL1/README.md](receiver/QrBinary/DenseL1/README.md)

## 历史说明

本仓库由两个独立仓库合并而来，两侧历史完整保留，提交日期为原值：

- 发送端源自 `markchoi321/Binary2QrCode`，历史路径已重写至 `sender/java/`
- 接收端源自 `markchoi321/QrBinary`，历史路径已重写至 `receiver/`

合并前的基线见 tag `v0.1.0-baseline`。
