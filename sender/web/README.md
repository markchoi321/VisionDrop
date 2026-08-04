# VisionDrop Web 发送端

纯原生 HTML + JS 实现的发送端，零依赖、零构建，双击 `dense.html` 即可运行
（或任意静态服务器托管）。协议语义与 `protocol/CONTRACT.md` 逐位一致。

## 文件

| 文件 | 内容 |
|---|---|
| `dense.html` | 密集 8 色发送端（选文件、档位、间隔、游标持久化、帧转储导出） |
| `vd-protocol.js` | 协议层：PRNG / SHA-256 / CRC32 / 量化 CDF / LT 与 RLNC 邻居 / 选参 / 档位 / 流层 / 会话 / 帧编码 |
| `vd-dense.js` | 密集 8 色 L1 编码器（VD-D1，含内置 GF(256) RS 编码，规约见 `protocol/dense/DENSE.md`） |
| `test/run-vectors.js` | Node 向量测试：比对 `protocol/vectors/` 的 L2–L5 部分，并导出帧转储 |
| `test/run-dense-vectors.js` | 密集模式向量测试：比对 `protocol/dense/vectors/` |

## 浏览器兼容性

只用 ES5 语法，不用 Promise / fetch / crypto.subtle / TextEncoder / BigInt /
箭头函数 / let。硬性下限由两个能力决定：

- `Uint8Array`（IE10+ / Android 4.0+ / Safari 5.1+）
- `FileReader.readAsArrayBuffer`（同上）

即 IE10、Android 4 原生浏览器、iOS 6 Safari 及之后的所有浏览器均可运行。
更旧的浏览器会得到明确的能力提示而不是白屏。`Math.imul` 缺失时自动降级为
手工半字乘法。文件导出在 IE10/11 走 `msSaveBlob`，其余走 Blob URL。

## 与桌面（Java）发送端的差异

- **不做压缩**：老浏览器没有原生 deflate 接口，流层 flags bit0 恒为 0。
  契约允许（压缩本就是自适应可选项），接收端按 flag 处理，无需区分来源。
  代价是文本类文件传输帧数变多；jpg/png/zip/mp4 等本就不可压缩，无差别。
- **播放状态存 localStorage**（键前缀 `vdweb.dense.`），语义对齐
  `PlaybackState`：游标只在帧真正显示后推进，同一文件同参数重传自动续接。

## 验证

```bash
# L2–L5 向量比对 + 导出帧转储（288 项检查）
node sender/web/test/run-vectors.js /tmp/vd_web_dump.bin

# 用参考实现端到端还原（含 30% 丢帧模拟）
python3 protocol/crosscheck.py /tmp/vd_web_dump.bin \
    --sha $(grep contentSha256 protocol/vectors/e2e.txt | awk '{print $2}') --drop 0.3
```

页面上的「导出帧转储」按钮产出契约第 14 节格式的转储文件，可直接交给
`crosscheck.py` 校验真实播放字节。

## 密集 8 色模式（唯一物理层）

`dense.html` 是密集 8 色物理层（VD-D1，借鉴 JAB Code 的 RGB 立方体调色板，
每格 3 bit）的发送端页面。载荷仍是标准 L2 帧。N=96 档单符号净载荷 2767 字节，
格子 96×96；N=128 档 5087 字节。

### 每档各自选 T

`pickParams` 接受可选的 `maxCapacity`：**不传就走遗留的默认容量表选块大小**，
密集页面必须传自己的容量，否则 T 是照着别的物理层挑的，套到色块码上会剩一截
装不满的尾巴——那截补零格渲染出来是纯黑，占版面不载信息。

更进一步，**每个档位各自选自己的 T**，而不是全档共用一个。帧长必须是
`20 + m*T`，单档的浪费就是 `(净容量 - 20) mod T`，一个 T 不可能让五档都装满。
某 1.2 MB 文件的实测：

| | N=64 | N=80 | N=96 | N=112 | N=128 |
|---|---|---|---|---|---|
| 单一 T=488（按默认容量表挑） | 1240 帧 / 89% | 827 帧 / 80% | 496 帧 / 89% | 355 帧 / 90% | 248 帧 / 96% |
| 每档各自选 T | 1203 帧 / 100% | 658 帧 / 100% | 442 帧 / 100% | 318 帧 / 100% | 240 帧 / 100% |

代价：T 变则 sessionId 变，所以**换档等于换会话，游标归零**。这在「换档本就是
重开一轮」的语义下是可接受的，档位下拉框直接显示每档的一轮帧数供选择。

注意 `pickParams` 的默认路径（不传 `maxCapacity`）被 `protocol/vectors/` 的
跨实现向量逐字段钉死，填充率平局判据只在显式传容量时生效——否则会挑出不同的
T，与 `refimpl.py` 对不上。

`vd-protocol.js` 里的 `QR_CAPACITY` 容量表与 `buildTiers()` 同理：**保留仅为
跨实现向量校验（`params.txt` / `tiers.txt`），色块码路径不使用**——`dense.html`
总是显式传 `maxCapacity`，`SendSession` 在传了容量时也不再构建这套档位表。

接收端已实现（`receiver/QrBinary/DenseL1/`），真机屏摄可整帧解出。
发送语义由 `protocol/dense/dense_refimpl.py` 的图像解码器验收：JS 编码器
渲染的符号在 gamma/白平衡/噪声/透视失真下解码，经喷泉码还原文件 SHA-256 一致。

```bash
node sender/web/test/run-dense-vectors.js       # 密集模式向量比对
python3 protocol/dense/dense_refimpl.py check   # 参考实现自检（含失真解码）
```
