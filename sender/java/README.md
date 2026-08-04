# VisionDrop 协议层交叉校验模块（Java）

**这不是一个可运行的发送端。** 二维码物理层已迁出本仓库，随之删除的还有本模块的
Swing 界面（`QRCodeGeneratorGUI`）、播放状态持久化（`PlaybackState`）与 ZXing 依赖。
剩下的是一份独立的 L2–L5 协议实现，唯一职责是给 `protocol/vectors/` 提供**第三方交叉校验**。

## 为什么留着

`protocol/vectors/` 是跨实现契约数据，目前由 Python 参考实现、Web 发送端、本模块三方
互验。L2–L5（帧层、喷泉码、流层、会话）与物理层无关，色块码路径原样沿用同一套语义，
因此这份 Java 实现继续有效。少一个独立实现就少一层保障，而维护成本只是一个不再运行的
Maven 模块。

协议语义以 `protocol/CONTRACT.md` 为唯一权威，`protocol/refimpl.py` 是参考实现。

## 测试

```bash
mvn -o test
```

`ProtocolVectorTest` 逐条比对 `protocol/vectors/` 下的向量文件。
离线环境下 surefire 的 junit4 provider 若无法解析，可直接运行等价入口：

```bash
mvn -o clean package -DskipTests
java -cp target/classes:target/test-classes org.file.qrcode.protocol.VectorCheck
```

## 代码结构

```
src/main/java/org/file/qrcode/protocol/     协议实现，逐函数对照 refimpl.py
├── Prng.java                 mix32 / xorshift32
├── RobustSoliton.java        量化对数 lnq + uint32 CDF + 抽度数
├── BlockComposer.java        blockId -> 参与异或的源块索引
├── LinearSolveComposer.java  解方程方案（RLNC）系数向量
├── PeelingComposer.java      剥洋葱方案（LT）邻居集合
├── ParamPicker.java          选参算法
├── QrCapacity.java           遗留容量表 + 档位生成（见下）
├── StreamCodec.java          L4 流层，raw deflate 自适应压缩
├── SessionId.java            会话标识确定性推导
├── SendSession.java          会话不可变参数 + 源块划分
└── FrameEncoder.java         整帧字节产出
```

## 关于 QrCapacity

`QrCapacity` 是**遗留容量表，仅用于跨实现向量校验，色块码路径不使用**。它对应 Web 端的
`QR_CAPACITY` 表，被 `ParamPicker` 用于选参，而选参与分档逻辑被 `protocol/vectors/params.txt`
和 `tiers.txt` 逐字段钉死。删掉它等于废掉这两个向量文件。色块码有自己的容量模型，
见 `protocol/dense/DENSE.md`。
