#!/usr/bin/env python3
"""VD-D1 密集彩色 L1 参考实现 + 向量生成器（规约见 DENSE.md）。

编码器是发送端的权威语义来源；图像解码器用于证明符号在光度/几何失真下
可解，并作为 iOS 接收端的行为参照（只要求字节一致，不要求实现细节一致）。

用法：
    python3 dense_refimpl.py gen      重新生成 vectors/
    python3 dense_refimpl.py check    自检（含失真图像解码与协议栈端到端）
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.join(HERE, "vectors")
sys.path.insert(0, os.path.join(HERE, ".."))
import refimpl as R                          # 复用 L2-L5 参考实现

# ---------------------------------------------------------------- 常量

GRID_SIZES = [64, 80, 96, 112, 128]
FINDER = 7                                   # 定位图形边长（格）
CAL_LEN = 8                                  # 定标条长度
HEADER_CELLS = 120                           # 40 bit x 3 重复
RS_PARITY = 32                               # 每块奇偶字节数
RS_DATA_MAX = 255 - RS_PARITY                # 每块数据字节上限 223
MAGIC = 0xD5
VERSION = 1
QUIET = 2                                    # 静区（格）

# 调色板：bit2=R bit1=G bit0=B
PALETTE = [(0, 0, 0), (0, 0, 255), (0, 255, 0), (0, 255, 255),
           (255, 0, 0), (255, 0, 255), (255, 255, 0), (255, 255, 255)]

# 调制方式（版本字节低 2 位）：0 = 8 色（每格 3 bit），1 = 4 色（每格 2 bit）
MOD_8 = 0
MOD_4 = 1
COLOR_COUNTS = (8, 4)
# 4 色档：2 bit 值 d -> 格值 v，取 {黑, 青, 品红, 黄}（立方体上互不相邻的四角）。
# v 的三位 = (d 高位, d 低位, 二者异或)。黑-黄、青-品红为 2 bit 对。
PALETTE4 = [0, 3, 5, 6]
INV4 = {v: d for d, v in enumerate(PALETTE4)}


def bits_per_cell(colors):
    if colors not in COLOR_COUNTS:
        raise ValueError("色数只能是 8 或 4: %r" % (colors,))
    return 3 if colors == 8 else 2

# ---------------------------------------------------------------- GF(256)

GF_EXP = [0] * 512
GF_LOG = [0] * 256
_x = 1
for _i in range(255):
    GF_EXP[_i] = _x
    GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
for _i in range(255, 512):
    GF_EXP[_i] = GF_EXP[_i - 255]


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return GF_EXP[GF_LOG[a] + GF_LOG[b]]


def gf_inv(a):
    return GF_EXP[255 - GF_LOG[a]]


def rs_gen_poly(degree):
    """首一生成多项式，最高次在前。g(x) = prod (x + a^d)。"""
    poly = [1]
    for d in range(degree):
        nxt = [0] * (len(poly) + 1)
        for i, co in enumerate(poly):
            nxt[i] ^= co
            nxt[i + 1] ^= gf_mul(co, GF_EXP[d])
        poly = nxt
    return poly


def rs_encode(data, degree):
    gen = rs_gen_poly(degree)
    res = [0] * degree
    for byte in data:
        factor = byte ^ res[0]
        res = res[1:] + [0]
        if factor:
            for k in range(degree):
                if gen[k + 1]:
                    res[k] ^= GF_EXP[(GF_LOG[gen[k + 1]] + GF_LOG[factor]) % 255]
    return bytes(res)


def rs_decode(block, degree):
    """带纠错的 RS 解码（BM + Chien + Forney）。返回数据部分，失败返回 None。"""
    n = len(block)
    poly = list(block)
    synd = []
    for i in range(degree):
        s = 0
        for co in poly:
            s = gf_mul(s, GF_EXP[i]) ^ co
        synd.append(s)
    if not any(synd):
        return bytes(block[:n - degree])
    # Berlekamp-Massey
    err_loc = [1]
    old_loc = [1]
    for i in range(degree):
        old_loc.append(0)
        delta = synd[i]
        for j in range(1, len(err_loc)):
            delta ^= gf_mul(err_loc[-(j + 1)], synd[i - j])
        if delta:
            if len(old_loc) > len(err_loc):
                new_loc = [gf_mul(c, delta) for c in old_loc]
                old_loc = [gf_mul(c, gf_inv(delta)) for c in err_loc]
                err_loc = new_loc
            scale = delta
            padded = [0] * (len(err_loc) - len(old_loc)) + old_loc
            err_loc = [e ^ gf_mul(scale, p) for e, p in zip(err_loc, padded)]
    while err_loc and err_loc[0] == 0:
        err_loc.pop(0)
    errs = len(err_loc) - 1
    if errs * 2 > degree:
        return None
    # Chien 搜索
    positions = []
    for i in range(n):
        x = GF_EXP[(255 - (n - 1 - i)) % 255]      # alpha^-(n-1-i)
        v = 0
        for co in err_loc:
            v = gf_mul(v, x) ^ co
        if v == 0:
            positions.append(i)
    if len(positions) != errs:
        return None
    # Forney
    synd_poly = list(reversed(synd))
    omega = [0] * (errs)
    prod = _poly_mul(synd_poly, err_loc)
    omega = prod[len(prod) - degree:][-errs:] if errs else []
    omega = prod[-degree:]                          # 取模 x^degree
    err_loc_deriv_src = err_loc[:-1]
    for pos in positions:
        xi = GF_EXP[(n - 1 - pos) % 255]
        xi_inv = gf_inv(xi)
        # omega(xi_inv)
        ov = 0
        for co in omega:
            ov = gf_mul(ov, xi_inv) ^ co
        # 形式导数 err_loc'(xi_inv)：奇数次项系数
        dv = 0
        deg = len(err_loc) - 1
        for idx, co in enumerate(err_loc):
            power = deg - idx
            if power % 2 == 1:
                term = gf_mul(co, _gf_pow(xi_inv, power - 1))
                dv ^= term
        if dv == 0:
            return None
        mag = gf_mul(ov, gf_inv(dv))
        mag = gf_mul(mag, xi)                       # fcr=0 修正因子
        poly[pos] ^= mag
    # 校验
    for i in range(degree):
        s = 0
        for co in poly:
            s = gf_mul(s, GF_EXP[i]) ^ co
        if s:
            return None
    return bytes(poly[:n - degree])


def _gf_pow(a, p):
    if p == 0:
        return 1
    if a == 0:
        return 0
    return GF_EXP[(GF_LOG[a] * p) % 255]


def _poly_mul(a, b):
    out = [0] * (len(a) + len(b) - 1)
    for i, x in enumerate(a):
        if x:
            for j, y in enumerate(b):
                out[i + j] ^= gf_mul(x, y)
    return out


# ---------------------------------------------------------------- 头部

def crc8(data):
    c = 0
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def build_header(payload_len, colors=8):
    mod = MOD_4 if bits_per_cell(colors) == 2 else MOD_8
    head = bytes([MAGIC, (VERSION << 4) | mod,
                  (payload_len >> 8) & 0xFF, payload_len & 0xFF])
    return head + bytes([crc8(head)])


def parse_header(head):
    """5 字节头部 -> (payloadLen, 色数)，校验失败抛异常。"""
    if head[0] != MAGIC or (head[1] >> 4) != VERSION or crc8(head[:4]) != head[4]:
        raise ValueError("头部校验失败")
    if head[1] & 0x0C or (head[1] & 0x03) not in (MOD_8, MOD_4):
        raise ValueError("未知调制方式 0x%02x" % head[1])
    colors = 4 if (head[1] & 0x03) == MOD_4 else 8
    return (head[2] << 8) | head[3], colors


# ---------------------------------------------------------------- 结构

CORNER = FINDER + 1                          # 角部功能块 8x8：定位图形 + 分隔带


def in_corner_block(r, c, N):
    return ((r < CORNER and c < CORNER) or (r < CORNER and c >= N - CORNER) or
            (r >= N - CORNER and c < CORNER) or
            (r >= N - CORNER and c >= N - CORNER))


def function_map(N):
    """返回 N*N 布尔列表，True = 功能格。行优先。"""
    fn = [False] * (N * N)

    def mark(r, c):
        fn[r * N + c] = True

    for r in range(N):
        for c in range(N):
            if in_corner_block(r, c, N):
                mark(r, c)
    for i in range(N):
        for (r, c) in ((0, i), (N - 1, i), (i, 0), (i, N - 1)):
            if not fn[r * N + c]:
                mark(r, c)
    for (r, c0) in cal_strips(N):
        for i in range(CAL_LEN):
            mark(r, c0 + i)
    return fn


def cal_strips(N):
    """四条定标条的 (row, 起始col)，紧贴角部功能块内侧。"""
    return [(CORNER, 1), (CORNER, N - 1 - CAL_LEN),
            (N - 1 - CORNER, 1), (N - 1 - CORNER, N - 1 - CAL_LEN)]


def free_cells(N):
    """自由格行优先列表 [(r,c), ...]。"""
    fn = function_map(N)
    return [(i // N, i % N) for i in range(N * N) if not fn[i]]


def capacity(N, colors=8):
    """返回 (free, dataCells, capBytes, netMax)。"""
    free = len(free_cells(N))
    data_cells = free - HEADER_CELLS
    cap_bytes = data_cells * bits_per_cell(colors) // 8
    net = 0
    p = cap_bytes
    while p > 0:
        if p + RS_PARITY * ((p + RS_DATA_MAX - 1) // RS_DATA_MAX) <= cap_bytes:
            net = p
            break
        p -= 1
    return free, data_cells, cap_bytes, net


# ---------------------------------------------------------------- 编码

def rs_blocks(payload):
    """分块并交织，返回交织后的字节流。"""
    P = len(payload)
    nb = (P + RS_DATA_MAX - 1) // RS_DATA_MAX
    base = P // nb
    rem = P % nb
    blocks = []
    off = 0
    for i in range(nb):
        ln = base + (1 if i >= nb - rem else 0)
        blocks.append(payload[off:off + ln])
        off += ln
    ecs = [rs_encode(b, RS_PARITY) for b in blocks]
    out = bytearray()
    max_len = base + (1 if rem else 0)
    for j in range(max_len):
        for b in blocks:
            if j < len(b):
                out.append(b[j])
    for j in range(RS_PARITY):
        for e in ecs:
            out.append(e[j])
    return bytes(out)


def encode_symbol(payload, N, colors=8):
    """载荷 -> N*N 格值数组（行优先，值 0-7）。4 色档的数据格只出现 PALETTE4。"""
    bpc = bits_per_cell(colors)
    _, _, _, net = capacity(N, colors)
    if len(payload) > net:
        raise ValueError("载荷超出容量: N=%d len=%d net=%d" % (N, len(payload), net))
    cells = [7] * (N * N)                    # 默认白（角部块的分隔带即白）

    def put(r, c, v):
        cells[r * N + c] = v

    # 定位图形（角部 8x8 块内的 7x7，分隔带保持白）
    corners = [(0, 0, 0), (0, N - FINDER, 0), (N - FINDER, 0, 0),
               (N - FINDER, N - FINDER, 4)]  # 右下角芯为红
    ring = ["1111111", "1000001", "1011101", "1011101",
            "1011101", "1000001", "1111111"]
    for (r0, c0, core) in corners:
        for dr in range(FINDER):
            for dc in range(FINDER):
                dark = ring[dr][dc] == "1"
                v = 0 if dark else 7
                if dark and 2 <= dr <= 4 and 2 <= dc <= 4:
                    v = core
                put(r0 + dr, c0 + dc, v)
    # 边框环（角部块之外）
    for i in range(N):
        for (r, c) in ((0, i), (N - 1, i), (i, 0), (i, N - 1)):
            if not in_corner_block(r, c, N):
                put(r, c, 0 if (r + c) % 2 == 0 else 7)
    # 定标条
    for (r, c0) in cal_strips(N):
        for i in range(CAL_LEN):
            put(r, c0 + i, i)
    # 头部 + 数据
    cells_free = free_cells(N)
    header_cells = cells_free[:HEADER_CELLS]
    data_cells = cells_free[HEADER_CELLS:]
    head = build_header(len(payload), colors)
    bits = []
    for byte in head:
        for b in range(7, -1, -1):
            bits.append((byte >> b) & 1)
    for k in range(3):
        for j in range(40):
            r, c = header_cells[j + 40 * k]
            put(r, c, 0 if bits[j] else 7)
    stream = rs_blocks(payload)
    bitpos = 0
    total_bits = len(stream) * 8
    for (r, c) in data_cells:
        v = 0
        for _ in range(bpc):
            bit = 0
            if bitpos < total_bits:
                bit = (stream[bitpos >> 3] >> (7 - (bitpos & 7))) & 1
            v = (v << 1) | bit
            bitpos += 1
        put(r, c, PALETTE4[v] if bpc == 2 else v)
    return cells


# ---------------------------------------------------------------- 矩阵解码

def decode_cells(cells, N, cells4=None):
    """从格值数组解回载荷（无图像环节），失败抛异常。

    cells4 可选：每格在 4 色组内的判决（0-3），图像解码器给出；
    缺省时按格值精确反查 PALETTE4（矩阵往返用）。"""
    cells_free = free_cells(N)
    header_cells = cells_free[:HEADER_CELLS]
    data_cells = cells_free[HEADER_CELLS:]
    bits = []
    for j in range(40):
        votes = 0
        for k in range(3):
            r, c = header_cells[j + 40 * k]
            votes += 1 if cells[r * N + c] < 4 else 0    # 黑=1（亮度判决）
        bits.append(1 if votes >= 2 else 0)
    head = bytearray(5)
    for j, b in enumerate(bits):
        if b:
            head[j >> 3] |= 0x80 >> (j & 7)
    payload_len, colors = parse_header(head)
    bpc = bits_per_cell(colors)
    stream_len = payload_len + RS_PARITY * ((payload_len + RS_DATA_MAX - 1) // RS_DATA_MAX)
    raw = bytearray((len(data_cells) * bpc + 7) // 8)
    bitpos = 0
    for (r, c) in data_cells:
        v = cells[r * N + c]
        if bpc == 2:
            if cells4 is not None:
                v = cells4[r * N + c]
            elif v in INV4:
                v = INV4[v]
            else:
                raise ValueError("4 色档出现组外格值 %d" % v)
        for b in range(bpc - 1, -1, -1):
            if (v >> b) & 1:
                raw[bitpos >> 3] |= 0x80 >> (bitpos & 7)
            bitpos += 1
    stream = bytes(raw[:stream_len])
    return deinterleave(stream, payload_len)


def deinterleave(stream, P):
    nb = (P + RS_DATA_MAX - 1) // RS_DATA_MAX
    base = P // nb
    rem = P % nb
    lens = [base + (1 if i >= nb - rem else 0) for i in range(nb)]
    blocks = [bytearray() for _ in range(nb)]
    pos = 0
    max_len = max(lens)
    for j in range(max_len):
        for i in range(nb):
            if j < lens[i]:
                blocks[i].append(stream[pos])
                pos += 1
    ecs = [bytearray() for _ in range(nb)]
    for j in range(RS_PARITY):
        for i in range(nb):
            ecs[i].append(stream[pos])
            pos += 1
    out = bytearray()
    for i in range(nb):
        got = rs_decode(bytes(blocks[i]) + bytes(ecs[i]), RS_PARITY)
        if got is None:
            raise ValueError("RS 纠错失败（块 %d）" % i)
        out += got
    return bytes(out)


# ---------------------------------------------------------------- 图像渲染

def render(cells, N, scale=8, quiet=QUIET):
    """格值 -> RGB 图像（list of rows, 每像素 (r,g,b)）。"""
    size = (N + quiet * 2) * scale
    img = [[(255, 255, 255)] * size for _ in range(size)]
    for r in range(N):
        for c in range(N):
            col = PALETTE[cells[r * N + c]]
            if col == (255, 255, 255):
                continue
            for y in range((r + quiet) * scale, (r + quiet + 1) * scale):
                row = img[y]
                for x in range((c + quiet) * scale, (c + quiet + 1) * scale):
                    row[x] = col
    return img


def _le32(buf, off, v):
    buf[off] = v & 0xFF
    buf[off + 1] = (v >> 8) & 0xFF
    buf[off + 2] = (v >> 16) & 0xFF
    buf[off + 3] = (v >> 24) & 0xFF


def write_bmp(path, img):
    h = len(img)
    w = len(img[0])
    row_bytes = (w * 3 + 3) // 4 * 4
    data = bytearray(54 + row_bytes * h)
    data[0:2] = b"BM"
    _le32(data, 2, len(data)); _le32(data, 10, 54); _le32(data, 14, 40)
    _le32(data, 18, w); _le32(data, 22, h)
    data[26] = 1; data[28] = 24
    _le32(data, 34, row_bytes * h)
    for y in range(h):
        off = 54 + (h - 1 - y) * row_bytes
        row = img[y]
        for x in range(w):
            r, g, b = row[x]
            data[off + x * 3] = b
            data[off + x * 3 + 1] = g
            data[off + x * 3 + 2] = r
    with open(path, "wb") as f:
        f.write(bytes(data))


# ---------------------------------------------------------------- 失真模拟

def screen_grid(img, period, depth=0.35):
    """模拟屏幕像素栅格（像素间的暗缝）。与相机采样率拍频即产生摩尔纹。"""
    h, w = len(img), len(img[0])
    out = []
    for y in range(h):
        row = img[y]
        orow = []
        vy = 1.0 - depth if (y % period) == 0 else 1.0
        for x in range(w):
            vx = 1.0 - depth if (x % period) == 0 else 1.0
            f = vx * vy
            p = row[x]
            orow.append((int(p[0] * f), int(p[1] * f), int(p[2] * f)))
        out.append(orow)
    return out


def resample_point(img, out_w, out_h):
    """最近邻降采样，模拟相机传感器的点采样。摩尔纹在这一步真正产生。"""
    h, w = len(img), len(img[0])
    out = []
    for y in range(out_h):
        sy = min(h - 1, int((y + 0.5) * h / out_h))
        row = img[sy]
        out.append([row[min(w - 1, int((x + 0.5) * w / out_w))] for x in range(out_w)])
    return out


def barrel(img, k1, seed=1):
    """径向（桶形/枕形）镜头畸变。k1 > 0 为枕形，< 0 为桶形。

    这是四角单应性无法表达的形变：符号边缘会向内或向外弓起，
    仅靠四个角点标定时，中部格心会系统性偏移。
    """
    h, w = len(img), len(img[0])
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    norm = math.sqrt(cx * cx + cy * cy)
    out = []
    for y in range(h):
        row = []
        for x in range(w):
            dx, dy = (x - cx) / norm, (y - cy) / norm
            r2 = dx * dx + dy * dy
            f = 1.0 + k1 * r2
            row.append(bilinear(img, cx + dx * f * norm, cy + dy * f * norm))
        out.append(row)
    return out


def distort(img, gamma=(1.0, 1.0, 1.0), gain=(1.0, 1.0, 1.0),
            noise=0, warp=0.0, seed=1, xtalk=None, lift=0):
    """光度失真（逐通道 gamma / 增益 / 噪声）+ 轻度透视。全部确定性。

    xtalk：3x3 通道串扰矩阵（行 = 输出通道），模拟屏幕基色在相机传感器上的
    交叉响应（真机实测绿基色在 R 通道读到 114）；lift：黑电平抬升（0-255），
    模拟环境光洗白造成的低对比度。二者在 gamma/增益之前作用，缺省不启用。"""
    h, w = len(img), len(img[0])
    st = R.mix32(seed)
    # 透视：四角各自向内/外偏移 warp*min(h,w) 像素
    m = min(h, w) * warp
    offs = []
    for _ in range(8):
        st = R.xorshift32(st)
        offs.append((st / 4294967296.0 * 2 - 1) * m)
    src_q = [(offs[0], offs[1]), (w - 1 + offs[2], offs[3]),
             (offs[4], h - 1 + offs[5]), (w - 1 + offs[6], h - 1 + offs[7])]
    H = homography([(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)], src_q)
    luts = []
    for ch in range(3):
        lut = []
        for v in range(256):
            x = (v / 255.0) ** gamma[ch] * gain[ch] * 255.0
            lut.append(int(max(0, min(255, x + 0.5))))
        luts.append(lut)
    out = []
    for y in range(h):
        row = []
        for x in range(w):
            sx, sy = apply_h(H, x, y)
            p = bilinear(img, sx, sy)
            if xtalk is not None or lift:
                q = p
                if xtalk is not None:
                    q = [sum(xtalk[ch][j] * p[j] for j in range(3)) for ch in range(3)]
                p = tuple(int(max(0, min(255, lift + v * (255 - lift) / 255.0 + 0.5)))
                          for v in q)
            st = R.xorshift32(st)
            n = ((st & 0xFFFF) / 65536.0 * 2 - 1) * noise
            row.append(tuple(
                int(max(0, min(255, luts[ch][p[ch]] + n))) for ch in range(3)))
        out.append(row)
    return out


def homography(dst, src):
    """4 点对 -> 3x3 单应性（dst -> src），高斯消元求解。"""
    A = []
    b = []
    for (dx, dy), (sx, sy) in zip(dst, src):
        A.append([dx, dy, 1, 0, 0, 0, -dx * sx, -dy * sx]); b.append(sx)
        A.append([0, 0, 0, dx, dy, 1, -dx * sy, -dy * sy]); b.append(sy)
    n = 8
    M = [A[i] + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[piv] = M[piv], M[col]
        d = M[col][col]
        M[col] = [v / d for v in M[col]]
        for r in range(n):
            if r != col and M[r][col]:
                f = M[r][col]
                M[r] = [v - f * M[col][i] for i, v in enumerate(M[r])]
    h = [M[i][8] for i in range(n)] + [1.0]
    return h


def apply_h(H, x, y):
    d = H[6] * x + H[7] * y + H[8]
    return ((H[0] * x + H[1] * y + H[2]) / d,
            (H[3] * x + H[4] * y + H[5]) / d)


def bilinear(img, x, y):
    h, w = len(img), len(img[0])
    if x < 0: x = 0.0
    if y < 0: y = 0.0
    if x > w - 1: x = float(w - 1)
    if y > h - 1: y = float(h - 1)
    x0, y0 = int(x), int(y)
    x1, y1 = min(x0 + 1, w - 1), min(y0 + 1, h - 1)
    fx, fy = x - x0, y - y0
    out = []
    for ch in range(3):
        a = img[y0][x0][ch] * (1 - fx) + img[y0][x1][ch] * fx
        b = img[y1][x0][ch] * (1 - fx) + img[y1][x1][ch] * fx
        out.append(int(a * (1 - fy) + b * fy + 0.5))
    return tuple(out)


# ---------------------------------------------------------------- 图像解码

def _luma(p):
    return (p[0] + 2 * p[1] + p[2]) / 4.0


def find_finders(img):
    """1:1:3:1:1 比例扫描，返回四个定位图形中心（按 左上/右上/左下/右下）。"""
    h, w = len(img), len(img[0])
    total = 0.0
    for y in range(0, h, 4):
        for x in range(0, w, 4):
            total += _luma(img[y][x])
    thresh = total / ((h // 4 + 1) * (w // 4 + 1) * 1.0) * 0.7 + 30
    cands = []
    for y in range(0, h, 2):
        runs = []                            # (值, 长度) 游程
        row = img[y]
        prev = _luma(row[0]) < thresh
        ln = 1
        xs = [0]
        for x in range(1, w):
            cur = _luma(row[x]) < thresh
            if cur == prev:
                ln += 1
            else:
                runs.append((prev, ln)); xs.append(x)
                prev, ln = cur, 1
        runs.append((prev, ln)); xs.append(w)
        for i in range(len(runs) - 4):
            seg = runs[i:i + 5]
            if not (seg[0][0] and not seg[1][0] and seg[2][0]
                    and not seg[3][0] and seg[4][0]):
                continue
            u = (seg[0][1] + seg[1][1] + seg[2][1] + seg[3][1] + seg[4][1]) / 7.0
            okp = all(abs(seg[k][1] - u * f) < u * 0.75
                      for k, f in ((0, 1), (1, 1), (2, 3), (3, 1), (4, 1)))
            if okp:
                cx = xs[i] + sum(s[1] for s in seg) / 2.0
                if _verify_v(img, int(cx), y, u, thresh):
                    cands.append((cx, float(y), u))
    if not cands:
        raise ValueError("未找到定位图形")
    # 聚类
    groups = []
    for (cx, cy, u) in cands:
        for g in groups:
            if abs(g[0] / g[2] - cx) < u * 4 and abs(g[1] / g[2] - cy) < u * 4:
                g[0] += cx; g[1] += cy; g[2] += 1
                break
        else:
            groups.append([cx, cy, 1])
    groups = sorted(groups, key=lambda g: -g[2])
    top = groups[0][2]
    pool = [(g[0] / g[2], g[1] / g[2]) for g in groups if g[2] * 2 >= top]
    if len(pool) < 4:
        raise ValueError("定位图形不足 4 个")
    # 数据区可能随机拼出 1:1:3:1:1 假阳性；真定位图形必然离图像四角最近，
    # 按四角就近认领，内部假阳性自然出局
    h_img, w_img = len(img), len(img[0])
    corners_img = [(0, 0), (w_img, 0), (0, h_img), (w_img, h_img)]
    out = []
    used = set()
    for (ix, iy) in corners_img:
        best, bd = None, None
        for k, (px, py) in enumerate(pool):
            if k in used:
                continue
            d = (px - ix) ** 2 + (py - iy) ** 2
            if bd is None or d < bd:
                best, bd = k, d
        used.add(best)
        out.append(pool[best])
    return out


def _verify_v(img, x, y, u, thresh):
    """竖向验证 1:1:3:1:1。y 应在黑芯内部，先上行穿过 芯/白环/外环 三段。"""
    h = len(img)
    lim = int(u * 5) + 2

    def dark(yy):
        return _luma(img[yy][x]) < thresh

    y0 = y
    steps = 0
    while y0 > 0 and dark(y0 - 1) and steps < lim:          # 芯顶
        y0 -= 1; steps += 1
    steps = 0
    while y0 > 0 and not dark(y0 - 1) and steps < lim:      # 白环顶
        y0 -= 1; steps += 1
    steps = 0
    while y0 > 0 and dark(y0 - 1) and steps < lim:          # 外环顶
        y0 -= 1; steps += 1
    runs = []
    yy = y0
    cur = True
    ln = 0
    while yy < h and len(runs) < 5:
        d = dark(yy)
        if d == cur:
            ln += 1
        else:
            runs.append(ln); cur = d; ln = 1
        yy += 1
    if len(runs) < 5:
        runs.append(ln)
    if len(runs) < 5:
        return False
    uu = sum(runs[:5]) / 7.0
    if uu <= 0:
        return False
    fac = (1, 1, 3, 1, 1)
    if not all(abs(runs[k] - uu * fac[k]) < uu * 1.0 for k in range(5)):
        return False
    # 中心 y 应落在第三段（黑芯）内
    core_top = y0 + runs[0] + runs[1]
    return core_top <= y < core_top + runs[2]


def decode_image(img, N):
    """图像 -> 载荷字节。"""
    pts = find_finders(img)
    cc = FINDER / 2.0                        # 定位图形中心的格坐标 3.5
    ref = [(cc, cc), (N - cc, cc), (cc, N - cc), (N - cc, N - cc)]
    H = homography(ref, pts)                 # 格坐标 -> 像素
    # 采样全部格中心
    samples = []
    for r in range(N):
        row = []
        for c in range(N):
            x, y = apply_h(H, c + 0.5, r + 0.5)
            row.append(bilinear(img, x, y))
        samples.append(row)
    # 定标：每通道置位/未置位均值取中点
    on = [[], [], []]
    off = [[], [], []]
    for (r, c0) in cal_strips(N):
        for i in range(CAL_LEN):
            p = samples[r][c0 + i]
            for ch, bit in ((0, 4), (1, 2), (2, 1)):
                (on if i & bit else off)[ch].append(p[ch])
    th = [(sum(on[ch]) / len(on[ch]) + sum(off[ch]) / len(off[ch])) / 2.0
          for ch in range(3)]
    gap = [max(1.0, (sum(on[ch]) / len(on[ch]) - sum(off[ch]) / len(off[ch])) / 2.0)
           for ch in range(3)]
    cells = [0] * (N * N)
    cells4 = [0] * (N * N)
    for r in range(N):
        for c in range(N):
            p = samples[r][c]
            v = ((1 if p[0] > th[0] else 0) << 2) | \
                ((1 if p[1] > th[1] else 0) << 1) | \
                (1 if p[2] > th[2] else 0)
            cells[r * N + c] = v
            # 4 色组内判决：归一化通道偏离与各组色的符号模式做相关，取最大
            u = [(p[ch] - th[ch]) / gap[ch] for ch in range(3)]
            best, bd = -1e9, 0
            for d, g in enumerate(PALETTE4):
                sc = sum(u[ch] if g & bit else -u[ch]
                         for ch, bit in ((0, 4), (1, 2), (2, 1)))
                if sc > best:
                    best, bd = sc, d
            cells4[r * N + c] = bd
    return decode_cells(cells, N, cells4)


# ---------------------------------------------------------------- 向量

def _write(name, lines):
    with open(os.path.join(VEC, name), "w") as f:
        f.write("\n".join(lines) + "\n")


def gen():
    os.makedirs(VEC, exist_ok=True)

    lines = ["# VD-D1 头部字节", "# 格式: <payloadLen> <hex10>"]
    for n in (0, 1, 20, 313, 1024, 4000, 65535):
        lines.append("%d %s" % (n, build_header(n).hex()))
    _write("dense_header.txt", lines)

    lines = ["# VD-D1 容量表", "# 格式: <N> <free> <数据格数> <容量字节> <净载荷上限>"]
    for N in GRID_SIZES:
        free, dc, cap, net = capacity(N)
        lines.append("%d %d %d %d %d" % (N, free, dc, cap, net))
    _write("dense_capacity.txt", lines)

    lines = ["# RS(数据+32奇偶) 向量", "# 格式: <数据hex> <奇偶hex>"]
    for n, seed in ((1, 3), (16, 7), (223, 11)):
        data = R._det_bytes(n, seed)
        lines.append("%s %s" % (data.hex(), rs_encode(data, RS_PARITY).hex()))
    _write("dense_rs.txt", lines)

    lines = ["# 整符号格值。第一行参数，第二行 N*N 个格值数字",
             "# 格式: <N> <payloadLen> <payload hex>"]
    for N, plen, seed in ((64, 313, 5), (96, 2000, 9)):
        payload = R._det_bytes(plen, seed)
        cells = encode_symbol(payload, N)
        lines.append("%d %d %s" % (N, plen, payload.hex()))
        lines.append("".join(str(v) for v in cells))
    _write("dense_symbol.txt", lines)

    # 4 色档：单独成文件，8 色向量保持逐字节不变
    lines = ["# VD-D1 4 色档容量表（每格 2 bit）",
             "# 格式: <N> <free> <数据格数> <容量字节> <净载荷上限>"]
    for N in GRID_SIZES:
        free, dc, cap, net = capacity(N, 4)
        lines.append("%d %d %d %d %d" % (N, free, dc, cap, net))
    _write("dense_capacity4.txt", lines)

    lines = ["# 4 色档整符号格值。第一行参数，第二行 N*N 个格值数字（数据格只含 0/3/5/6）",
             "# 格式: <N> <payloadLen> <payload hex>"]
    for N, plen, seed in ((64, 313, 5), (96, 1500, 9)):
        payload = R._det_bytes(plen, seed)
        cells = encode_symbol(payload, N, 4)
        lines.append("%d %d %s" % (N, plen, payload.hex()))
        lines.append("".join(str(v) for v in cells))
    _write("dense_symbol4.txt", lines)
    print("向量已写入 %s" % VEC)


# ---------------------------------------------------------------- 自检

def check():
    print("=== 容量表 ===")
    for colors in COLOR_COUNTS:
        for N in GRID_SIZES:
            free, dc, cap, net = capacity(N, colors)
            print("  %d色 N=%-4d 自由格 %-6d 容量 %-5dB 净载荷 %-5dB (QR V40 的 %.1f 倍)"
                  % (colors, N, free, cap, net, net / 2953.0))

    print("\n=== 矩阵往返 ===")
    for colors in COLOR_COUNTS:
        for N in GRID_SIZES:
            _, _, _, net = capacity(N, colors)
            payload = R._det_bytes(net, N)
            cells = encode_symbol(payload, N, colors)
            assert decode_cells(cells, N) == payload
            print("  %d色 N=%d 满载荷 %dB 往返一致" % (colors, N, net))
    # 4 色档数据格只能出现选定四色；头部调制位受 crc8 保护
    cells = encode_symbol(R._det_bytes(500, 1), 64, 4)
    data = [cells[r * 64 + c] for (r, c) in free_cells(64)[HEADER_CELLS:]]
    assert set(data) <= set(PALETTE4)
    h = bytearray(build_header(500, 4))
    assert h[1] == 0x11
    h[1] ^= 0x01
    try:
        parse_header(bytes(h))
        assert False, "调制位翻转未被 crc8 拦下"
    except ValueError:
        pass
    print("  4 色数据格只含 %s；调制位翻转被 crc8 拦下" % PALETTE4)

    print("\n=== 干净图像解码 ===")
    N = 96
    imgs = {}
    for colors in COLOR_COUNTS:
        _, _, _, net = capacity(N, colors)
        payload = R._det_bytes(net, 42)
        cells = encode_symbol(payload, N, colors)
        img = render(cells, N, scale=8)
        assert decode_image(img, N) == payload
        imgs[colors] = (img, payload)
        print("  %d色 N=%d scale=8 一致" % (colors, N))

    print("\n=== 失真图像解码 ===")
    cases = [
        ("gamma 0.85/1.2/1.1", dict(gamma=(0.85, 1.2, 1.1))),
        ("增益 0.9/1.05/0.95", dict(gain=(0.9, 1.05, 0.95))),
        ("噪声 +-10", dict(noise=10, seed=3)),
        ("透视 0.8%", dict(warp=0.008, seed=5)),
        ("组合失真", dict(gamma=(0.9, 1.15, 1.05), gain=(0.95, 1.02, 0.9),
                          noise=8, warp=0.006, seed=7)),
    ]
    for colors in COLOR_COUNTS:
        img, payload = imgs[colors]
        for label, kw in cases:
            got = decode_image(distort(img, **kw), N)
            assert got == payload, label
            print("  %d色 %s 一致" % (colors, label))

    print("\n=== 协议栈端到端（L2 帧过密集 L1）===")
    content = R._det_bytes(20000, 2026)
    stream, _ = R.build_stream(content, "fixture.bin", allow_compress=False)
    T = R.T_MIN_LT
    K = -(-len(stream) // T)
    _, _, _, net = capacity(96)
    m = (net - R.FRAME_HEADER_LEN) // T
    sid = R.session_id(__import__("hashlib").sha256(content).digest(), T, K, R.CODEC_LT)
    cdf = R.rs_cdf_q(K)
    src = R.split_source_blocks(stream, K, T)
    dec = R.PeelingDecoder(K, T, cdf)
    base = frames = 0
    st = R.mix32(99)
    while not dec.complete:
        frame = R.encode_frame(src, K, T, R.CODEC_LT, False, sid, base, m, cdf)
        sym = encode_symbol(frame, 96)
        img2 = distort(render(sym, 96, scale=8),
                       gamma=(0.92, 1.1, 1.0), noise=6, warp=0.004,
                       seed=base + 1)
        st = R.xorshift32(st)
        if (st / 4294967296.0) >= 0.2:        # 模拟 20% 丢帧
            got = decode_image(img2, 96)
            p = R.parse_frame(got)
            assert p is not None
            for i, blk in enumerate(p[7]):
                dec.add((base + i) & 0xFFFFFFFF, blk)
        base += m
        frames += 1
        assert frames < 200
    out = dec.assemble()
    name, _, _, digest, _, recovered = R.parse_stream(out)
    assert recovered == content
    print("  %d 帧（含 20%% 丢帧模拟，每帧 %d 块）还原一致，SHA-256 通过" % (frames, m))
    print("\n全部自检通过")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "gen":
        gen()
    else:
        check()
