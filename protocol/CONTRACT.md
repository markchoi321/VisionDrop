# VisionDrop 协议 v1 实现契约

本文件把 `docs/refactor-design.md` 中未完全确定的算法细节固定下来，是发送端与接收端
唯一的权威语义来源。凡设计文档与本文件冲突，以本文件为准，冲突点在第 10 节列出。

**范围限于 L2–L5**（帧层、喷泉码、流层、会话）。物理层 L1 是密集 8 色色块码（VD-D1），
规约另见 `protocol/dense/DENSE.md`；二维码物理层已迁出本仓库，本文件不再描述它。
文中残留的 QR 版本/容量概念只作为**遗留容量表**存在，见第 1、5、9 节的说明。

参考实现：`protocol/refimpl.py`。测试向量：`protocol/vectors/`。
**两端实现必须逐位复现向量，这是唯一的验收标准。**

---

## 1. 常量

| 名称 | 值 | 出处 |
|---|---|---|
| 帧魔数 | `0x56` | 设计 4.1 |
| 协议版本 | `1`（flags 高 4 位） | 设计 4.2 |
| 帧头长度 | **20** 字节 | 设计 4.1 |
| 块头长度 | **0**（编码块无头） | 设计 4.3 |
| 流魔数 | `"VD"` = `0x56 0x44` | 设计 6.1 |
| 流格式版本 | `1` | 设计 6.1 |
| 流头定长部分 | 54 字节 + 文件名 | 设计 6.1 |
| 块大小 T 范围 | `16 … 500` | 设计 8.3 约束三 |
| 解方程块数上限 | `K ≤ 2720` 且 `K ≤ 8×T` | 设计 8.3 约束一、二 |
| 剥洋葱块大小下限 | `T ≥ 293` | 设计 8.3 约束四 |
| 最低档容量 | 512 字节 | 设计 10.4 |
| 最大档位数 | 10 | 设计 10.4 |
| LT 度分布参数 | `c = 0.03`, `δ = 0.05` | 见 10.2 |

### 遗留容量表

`refimpl.py` 的 `QR_CAPACITY`（版本 1–40，模块数 = `17 + 4 × 版本`，上限 2953 字节）是
**遗留容量表，仅用于跨实现向量校验，色块码路径不使用**。它是第 5 节选参与第 9 节档位
生成的输入，而这两处逻辑被 `vectors/params.txt` 与 `vectors/tiers.txt` 逐字段钉死，
因此实现该表仍是通过向量的前提。VD-D1 有自己的容量模型，见 `protocol/dense/DENSE.md`。

`m` 是 uint8，`T ≥ 16` 保证 `m = (2953−20)/T ≤ 183`，不会溢出。

---

## 2. PRNG（设计 5.4）

```
uint32 mix32(uint32 x):            // lowbias32 finalizer
    x ^= x >> 16;  x *= 0x7feb352d
    x ^= x >> 15;  x *= 0x846ca68b
    x ^= x >> 16
    return x

uint32 xorshift32(uint32 s):
    s ^= s << 13
    s ^= s >> 17
    s ^= s << 5
    return s
```

- **Java**：用 `int` 承载，所有右移必须写 `>>>`，乘法自然溢出即模 2³²。
- **Swift**：用 `UInt32`，乘法用 `&*`，左移用 `&<<`（或先掩码）。
- 禁止使用 `java.util.Random` / `SystemRandomNumberGenerator` / 任何语言内置 RNG。

测试向量：`vectors/prng.txt`。

---

## 3. L3 解方程方案（RLNC，codec = 0）

### 3.1 系数向量

```
bits(blockId, K):
    base = mix32(blockId)
    bits = 空的 K 位位集
    对 i = 0, 1, 2, …，直到覆盖 K 位：
        word = mix32(base XOR (i × 0x9E3779B9))     // 32 位乘法自然溢出
        把 word 的 32 位放到位集的 [32i, 32i+32) 区间
    截断到 K 位
    若 bits 全零：bits = 只有第 (blockId mod K) 位为 1        // ← 兜底，见下
    返回 bits
```

位集第 `p` 位为 1 表示源块 `p` 参与异或。

- **严禁改用 xorshift32 连续输出拼接**。它是 GF(2) 线性变换，所有向量落在 32 维子空间内，
  秩永远停在 32，解码永不收敛（设计 5.2，该缺陷已被实际触发过）。
- **全零兜底是新增规则**，设计文档没有。K 很小时（例如退化的 K=1）全零向量出现概率不可忽略，
  而全零块不携带任何信息。两端必须实现同一条兜底规则。

字节序约定（仅用于测试向量的表示）：位集按 **小端** 序列化，字节 `j` 的第 `b` 位对应源块
索引 `8j + b`。

测试向量：`vectors/rlnc_coeff.txt`。

### 3.2 解码

增量高斯消元，见 `refimpl.py` 的 `LinearSolveDecoder`：

1. 收到 `(blockId, payload)`，去重后算出系数向量 `c`、数据 `d`。
2. 循环：取 `c` 的最高位 `p`；若 `basis[p]` 存在则 `c ^= basis[p].c`、`d ^= basis[p].d`；
   否则 `basis[p] = (c, d)` 并结束（秩 +1）。
3. `c` 变为全零表示该块线性相关，丢弃。
4. 秩达到 `K` 后回代：按 pivot 升序把上三角化简为单位阵，`basis[i].d` 即源块 `i`。

内存：系数矩阵 `K × ceil(K/8)` 字节，约束二保证它不超过数据本身。

---

## 4. L3 剥洋葱方案（LT，codec = 1）

### 4.1 Robust Soliton 度分布

```
ρ(1) = 1/K;  ρ(d) = 1/(d(d−1))            d = 2…K
R = c × lnq(K/δ) × sqrt(K)
kr = max(1, floor(K/R))
τ(d) = R/(d×K)                            d = 1…kr−1
τ(kr) = max(0, R × lnq(R/δ) / K)          仅当 kr ≤ K
τ(d) = 0                                  其余
Z = Σ_{d=1..K} (ρ(d) + τ(d))              // 按 d 升序累加，顺序不可改
```

**`lnq(x) = floor(ln(x) × 65536 + 0.5) / 65536`** —— 量化对数。这是对设计 5.5 的加固：
跨平台 `libm` 的 `ln` 可能有 ulp 级差异，若差异传导到 CDF，两端会在极少数编码块上抽出
不同的度数，产生一个坏块，经 BP 的 XOR 链污染大片源块，且帧 CRC 无法发现。量化到 2⁻¹⁶
网格后，`ln` 的实现差异被挡在网格外，其余全部是 IEEE 双精度加减乘除，逐位确定。

### 4.2 CDF 量化与抽样

```
acc = 0
对 d = 1…K：                               // 升序累加，顺序不可改
    acc += (ρ(d) + τ(d)) / Z
    cdf[d] = floor(acc × 2³²)，上限截到 2³²
cdf[K] = 2³²                               // 末档兜底，保证一定命中
```

`cdf` 存 **uint64 或 int64**（因为 `2³²` 放不进 uint32）。抽样全程整数比较：

```
sampleDegree(state, K):
    state = xorshift32(state)
    r = state                              // uint32
    二分找最小的 d ∈ [1, K] 使 cdf[d] > r
    返回 (d, state)
```

会话建立时预计算一次，两端各自计算，不传输。

### 4.3 邻居集合

```
neighborsOf(blockId, K):
    state = mix32(blockId)
    (d, state) = sampleDegree(state, K)
    picked = 空集合
    while |picked| < d:                    // 拒绝采样
        state = xorshift32(state)
        picked.insert(state mod K)         // Java 必须做无符号取模
    返回 picked 升序排列
```

- `state mod K` 有模偏，但两端一致且 `K << 2³²`，偏差可忽略。
- 不做任何度数截断。`d` 接近 `K` 时拒绝采样期望迭代 `K·ln K` 次，概率极低且代价可接受。
- **Java 无符号取模**：`(int)(Integer.toUnsignedLong(state) % K)`。

测试向量：`vectors/rs_cdf.txt`、`vectors/lt_neighbors.txt`。

### 4.4 解码

剥离（BP）解码，见 `refimpl.py` 的 `PeelingDecoder` 与设计 5.6。必须维护倒排索引
`源块索引 → pending 下标集合`，否则传播步退化为全表扫描。

---

## 5. 选参算法（设计 8.3）

```
输入 streamLen，输出 (codec, T, K)
候选 = []
for T in 16…500:
    K = ceil(streamLen / T)                 // 至少 1
    m = (2953 − 20) / T                     // 整除，按遗留容量表上限估算
    if m < 1: continue
    if K ≤ 2720 且 K ≤ 8×T:
        frames = ceil( ceil(K × (1 + 2.0/K)) / m )
        候选 += (frames, T, 解方程, K, m)
    if T ≥ 293:
        frames = ceil( ceil(K × (1 + 1.85/K^0.37)) / m )
        候选 += (frames, T, 剥洋葱, K, m)
按 (frames, T, codec) 升序排序，取第一个        // codec: 解方程=0 < 剥洋葱=1
```

`K ≤ 1` 时两个 ε 均取 0。

**并列取舍顺序（帧数 → T 最小 → 解方程优先）是本契约固定的**，设计文档未写明。
`T` 最小优先是为了换取更细的档位划分与更高的中高档载荷率（设计 8.5：500 KB 时 T=293 与
T=419 都是 175 帧，选 293）。

**不实现独立的"直传模式"。** 设计 8.6 自己指出直传是解方程的自然退化，独立实现只会多一条
代码路径。小文件走同一套选参即可（5 KB → T=26, K≈200, ε=1%）。

输出对照：`vectors/params.txt`，与设计 8.4 分档表逐行一致。式中的 2953 / 20 取自第 1 节的
遗留容量表：**该选参算法是跨实现向量校验的对象，色块码路径按每档自己的净载荷另行选 T**
（见 `protocol/dense/DENSE.md`）。

---

## 6. L4 流层（设计 6.1 / 6.3）

```
Stream = StreamHeader ‖ Payload

StreamHeader（大端）
 0   2  magic         "VD"
 2   1  version       1
 3   1  flags         bit0 = compressed，其余置 0
 4   8  originalSize  原始文件字节数 uint64
12   8  payloadSize   Payload 字节数 uint64
20  32  sha256        原始文件内容的 SHA-256
52   2  fileNameLen   uint16
54   n  fileName      UTF-8

Payload = rawDeflate(原始文件) 若 compressed，否则 = 原始文件
```

压缩：**raw deflate，无 zlib/gzip 容器**。

- Java：`new Deflater(Deflater.BEST_COMPRESSION, true)`（第二参数 `nowrap=true`）
- Swift：zlib `deflateInit2(..., windowBits = -15, ...)` / `inflateInit2(..., -15)`，
  或 Compression framework 的 `COMPRESSION_ZLIB`（该常量产出的正是 raw deflate）
- **自适应**：`压缩后长度 >= 原始长度` 时不压缩并清除 flag。对 jpg/png/zip/mp4 这是必需的，
  deflate 会让它们变大。

**压缩输出的字节不要求跨实现一致**（只有解压结果必须一致）。因此帧字节测试向量一律
`compressed = 0`。接收端的 inflate 正确性由 `vectors/deflate_plain.bin` /
`vectors/deflate_packed.bin` 这一对文件验证：inflate(packed) 必须逐字节等于 plain。

---

## 7. L5 会话标识（设计 7.2）

```
sessionId = SHA256( sha256(文件内容) ‖ T(2B 大端) ‖ K(3B 大端) ‖ codec(1B) ) 的前 4 字节
          解释为大端 uint32
```

`codec` 字节：解方程 = 0，剥洋葱 = 1。

---

## 8. L2 帧层（设计 4.1）

```
帧头（20 字节，大端）
 0   1  magic         0x56
 1   1  flags         bit7-4 版本(=1)  bit3 compressed  bit2 codec  bit1-0 保留=0
 2   4  sessionId
 6   3  K             uint24
 9   2  T             uint16
11   1  m             uint8
12   4  baseBlockId   uint32
16   4  crc32         覆盖偏移 20 到帧尾

编码块数组：m 个，每个 T 字节，无块头。第 i 块的序号 = baseBlockId + i（uint32 回绕）
帧总长 = 20 + m × T
```

- CRC32 是标准 IEEE / zlib CRC32（Java `java.util.zip.CRC32`，Swift zlib `crc32`）。
- 接收端校验顺序：长度 ≥ 20 → magic → 协议版本 → 长度等于 `20 + m×T` → CRC32。
  任一失败即整帧丢弃并计入 `framesRejected`。
- **CRC 不可省略**：BP 解码中一个坏块会沿 XOR 链污染大片已解出的源块，且事后无法定位。

编码块载荷 = `neighborsOf(blockId)` 指定的源块按索引逐字节异或。
源块划分：`padded = stream ‖ 0x00 × (K×T − streamLen)`，第 `i` 块 = `padded[i×T … (i+1)×T)`。

测试向量：`vectors/stream.bin`（源）、`vectors/frames_rlnc.txt`、`vectors/frames_lt.txt`。
`vectors/e2e.txt` 给出对应的 T / K / m / sessionId。

---

## 9. 档位生成（设计 10.4）

**遗留算法，仅用于跨实现向量校验**：输入是第 1 节的遗留容量表，输出被 `vectors/tiers.txt`
钉死。色块码的档位是物理层参数 N=64–128，不走这套规则，见 `protocol/dense/DENSE.md`。

```
need = max(512, 20 + T)
候选 = 所有满足 capacity(V) ≥ need 且 m = (capacity(V) − 20)/T ≥ 1 的版本
同一个 m 只保留最小的版本
按 m 升序排列
若档数 > 10：按 m 的对数等比抽取 10 档
    targets[j] = m_min × (m_max/m_min)^(j/9)，j = 0…9
    对每个 target 取第一个 m ≥ target 的档；若已被选中则顺延到下一个
    仍不足 10 档时，从尾部向前回填未选中的档
    最终按 m 升序排列
```

`T = 293` 时该规则恰好产出设计 8.5 的十档（V15/17/21/25/28/31/34/36/38/40，m = 1…10）。

测试向量：`vectors/tiers.txt`。

**换档只改变 `m`**：`T`、`K`、`codec`、`sessionId` 全部不变，接收端已收到的编码块继续有效。

---

## 10. 与设计文档的差异清单

| # | 设计文档 | 本契约 | 理由 |
|---|---|---|---|
| 10.1 | 4.4 写 `avail = P − 16`、`m = floor(avail/(4+T))` | `m = floor((P − 20)/T)` | 4.4 是 16 字节帧头 + 4 字节块头时代的残留；4.1/4.3/8.1/8.3/10.4 已统一为 20 字节帧头、块无头。取后者 |
| 10.2 | 5.5 初始 `c = 0.05` | `c = 0.03` | 8.2 实测最优 `c ≈ 0.02–0.03`，且 ε 拟合式 `1.85/K^0.37` 就是按各 K 最优 c 拟合的。用 0.05 会与拟合式不自洽 |
| 10.3 | 5.5 CDF 用 double + 浮点二分 | 量化对数 + uint32 整数 CDF | 消除跨平台 `ln` 的 ulp 差异导致的双端度数不一致，见 4.1 |
| 10.4 | 5.2 未规定全零系数向量 | 强制置位 `blockId mod K` | 全零块不携带信息，小 K 时概率不可忽略 |
| 10.5 | 8.3 未规定并列取舍 | 帧数 → T 最小 → 解方程优先 | sessionId 依赖选参结果，必须确定性 |
| 10.6 | 8.6 单列"直传模式" | 不实现 | 8.6 自述是解方程的自然退化，独立实现只增代码路径 |
| 10.7 | 5.3 未规定索引抽样方式 | 拒绝采样 + `state mod K` | 需要一个两端可逐位复现的确定算法 |
| 10.8 | 9.2 stats 40 字节未展开 | 见 11 节字段表 | 需要精确布局 |

---

## 11. 进度持久化格式（设计 9.2）

文件名 `Documents/progress/<sessionId 8位小写hex>.vdpg`，每会话一个文件。全部大端。

```
文件头
 0   4  magic          "VDPG"
 4   2  formatVersion  1
 6   2  sessionCount   uint16（单会话文件恒为 1，保留多会话打包能力）

会话记录 × sessionCount
 0   4  sessionId
 4   3  K              uint24
 7   2  T              uint16
 9   1  flags          bit0 compressed, bit1 有 meta, bit2 codec
10   2  metaLen        uint16，无 meta 时为 0
12   …  meta           StreamHeader 原样字节（54 + fileNameLen）
 …   4  solvedCount    uint32
 …   …  solvedBlocks   solvedCount × (3B 源块索引 + T 字节数据)
 …   4  pendingCount   uint32
 …   …  pendingBlocks  见下
 …  40  stats          见下

pendingBlock 记录
 0   4  blockId
 4   2  degree         剩余邻居数 uint16
 6   …  neighbors      degree × 3 字节索引（升序）
 …   T  payload        残值

stats（40 字节）
 0   4  framesAccepted   uint32
 4   4  framesRejected   uint32
 8   4  blocksReceived   uint32
12   4  blocksDuplicate  uint32
16   4  maxBlockId       uint32
20   4  reserved         置 0
24   8  firstSeenAt      float64，Unix 时间戳秒
32   8  lastSeenAt       float64，Unix 时间戳秒
```

**解方程会话的 pendingBlocks 语义不同**：存的是消元基（basis），`blockId` 字段写入该基行的
pivot 索引，`neighbors` 写系数向量中置 1 的源块索引（升序），`payload` 写该基行的数据。
恢复时直接重建 `basis[pivot] = (系数向量, 数据)`，不需要重新推导。

`neighbors` 可由 `blockId` 与 K 重算，存下来是为省去恢复时的重算，并可作为 PRNG
跨版本一致性的自检：恢复时重算并比对，不一致则丢弃该 pending 块并记日志。
（解方程会话不做此自检，因为存的是消元后的基而非原始块。）

策略：
- 自动保存：会话新增源块且距上次保存超过 5 秒；App 进入后台时立即全量保存
- 恢复：App 启动时扫描目录加载全部会话
- 清理：会话完成并成功落盘文件后删除对应进度文件；提供手动清理入口
- 导出/导入（跨设备迁移）复用同一格式，`sessionCount` 可 > 1

---

## 12. 兼容性（设计 12.1）

接收端按载荷首字节分流：`0x56` 走新二进制协议，`0x7B`（`{`）走旧 JSON 分支。
旧分支只保留解析与重组能力，不再新增功能。

L1 交给上层的一律是**完整帧字节**（首字节即帧魔数 `0x56`）。物理层如何从符号里取出这串
字节属于 L1 的内部事务，本契约不作规定；VD-D1 的取字节路径见 `protocol/dense/DENSE.md`。

---

## 13. 测试向量文件清单

| 文件 | 内容 | 格式 |
|---|---|---|
| `prng.txt` | mix32 / xorshift32 | `<函数> <输入hex8> <输出hex8>` |
| `rs_cdf.txt` | 量化 CDF 抽样点 | `<K> <d> <cdf_q 十进制>` |
| `lt_neighbors.txt` | LT 邻居集合 | `<K> <blockId> <degree> <idx,idx,…>` |
| `rlnc_coeff.txt` | RLNC 系数向量 | `<K> <blockId> <hex 小端>` |
| `params.txt` | 选参输出（遗留容量表，见第 1 / 5 节） | `<streamLen> <codec> <T> <K> <m@上限> <frames>` |
| `tiers.txt` | 档位表（遗留容量表，见第 1 / 9 节） | `<T> <档位序号> <版本> <容量> <m>` |
| `deflate_plain.bin` / `deflate_packed.bin` | raw deflate 往返 | 二进制 |
| `fixture.bin` | 20000 字节原始文件 | 二进制 |
| `stream.bin` | 上者的流层字节（未压缩） | 二进制 |
| `frames_rlnc.txt` / `frames_lt.txt` | 整帧字节 | `<baseBlockId> <整帧hex>` |
| `e2e.txt` | fixture 的 T/K/m/sessionId/sha256 | `<键> <值>` |

`#` 开头的行是注释，需跳过。

重新生成：`python3 protocol/refimpl.py gen`；参考实现自检：`python3 protocol/refimpl.py check`。

## 14. 帧转储格式（跨语言校验用）

两端都应提供一个「把连续若干帧导出到文件」的调试出口，格式是连续的
`[4 字节大端长度][帧字节]` 记录，无文件头。用参考实现校验：

```
python3 protocol/crosscheck.py <dump> --sha <原始文件sha256> [--name <文件名>] [--drop 0.3]
```

它会用参考实现的解码器还原并比对 SHA-256，退出码 0 表示一致。这是验证
「一端产出的真实字节能被另一端正确解读」的手段，比双方各自跑向量更强。
