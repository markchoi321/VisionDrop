/*
 * VD-D1 密集彩色 L1 解码核心实现。规约：protocol/dense/DENSE.md。
 *
 * 流程（与 dense_refimpl.py 第 8 节一致）：
 *   亮度游程扫描(1:1:3:1:1) -> 四角单应性 -> 定标条调色板 -> 头部多数表决 ->
 *   抽样判决（坏帧 / 重复帧早退）-> 逐格采样 -> 整体偏移判决 ->
 *   分区光照修正再判 -> 解交织 -> RS errata(错误+擦除)纠错 -> L2 帧字节
 */
#include "vd_dense_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * 分阶段计时。clock_gettime 单次约几十纳秒，一帧七八次可以忽略，因此无条件
 * 开着——需要的时候没有数据，比常态多花这点时间糟糕得多。累加器是线程局部的，
 * 与文件里其他可变 static 的处置一致（见 README 的线程安全一节）。
 */
static double clk_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}
static _Thread_local double g_ms_geom, g_ms_calib;

/* ---------------------------------------------------------------- 常量 */

#define FINDER 7
#define CORNER 8                /* 角部功能块：定位图形 + 分隔带 */
#define CAL_LEN 8
#define HEADER_CELLS 120
#define RS_PARITY 32
#define RS_DATA_MAX 223

/*
 * 单块最多接受的擦除数。理论上限是 RS_PARITY(32)，这里刻意留 8 个校验符号的余量。
 *
 * 擦除数逼近 32 时，纠错末尾那道「重算综合征」的复验会失效：32 个擦除位置
 * 配 32 个校验符号，方程组恰定，**无论真实错误在哪，都必然解出一个合法码字**。
 * 于是解码器报成功、码字自洽，字节却是错的。真机 N=128 照片实测正是如此：
 * 22 个块全部「纠错成功」，重组出的 L2 帧却 magic、T、m 各错一位，
 * 两张不同照片解出的 CRC 字段还完全相同——典型的伪成功。
 *
 * 留 8 个符号不参与求解，伪成功概率降到约 2^-64。代价是擦除能力从 32 降到 24，
 * 仍高于纯错误模式的 16，而擦除本就是额外收益、不是保底。
 */
#define RS_ERAS_MAX (RS_PARITY - 8)
#define MAGIC 0xD5
#define VERSION 1
/*
 * 调制方式：头部版本字节低 2 位，0 = 8 色（每格 3 bit），1 = 4 色（每格 2 bit）。
 * 4 色只用 {黑, 青, 品红, 黄} 四角，2 bit 值 d 对应格值 PALETTE4[d]。
 * 判决时 8 色在 COLORS8 里找最近邻、返回格值；4 色在 PALETTE4 里找、返回 d，
 * 两者都恰好是「要写进位流的那几位」。
 */
#define MOD_4 1
static const int COLORS8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const int PALETTE4[4] = { 0, 3, 5, 6 };
#define N_MAX 128
#define MAX_CAND 1024           /* 定位图形候选上限 */
#define MAX_GROUP 256
/*
 * 定位图形的最小模块尺寸（像素）。低于此值一律不算候选。
 *
 * 缺了这道门限会出事：比例容差是 u×0.75，u 很小时容差跟着掉到亚像素级，
 * 任何噪点都能通过 1:1:3:1:1 检验。真机 4032×3024 照片实测检出 128 组候选
 * 全是 u≈1–3px 的噪点，把候选数组占满，扫到真符号时已无槽位可用，
 * 于是被静默丢弃——表现为「定位不到」或「认领到界面图标」。
 *
 * 取 2.5：最密的档位也需要约 3px/格才可能解得出，低于此的候选本就无用。
 */
#define MIN_MODULE_PX 2.5

static const int GRID_SIZES[] = { 64, 80, 96, 112, 128 };
#define N_GRIDS 5

/* 判决阈值估计的亚采样上限。步长按 n/26 取，样本数落在 600-1500，留一倍余量 */
#define CALIB_MAX_SAMPLES 3000

/* ---------------------------------------------------------------- GF(256) */

/*
 * 本文件里所有可变的 static 缓冲都必须是线程局部的。
 * 解码本身是纯函数式的（输入图像 -> 输出字节），但这些缓冲一旦被两个线程
 * 同时用就会互相踩。App 里解码跑在单条串行队列上，所以从没暴露；
 * 而 Swift Testing 默认并行跑用例，两个解码用例撞在一起就随机失败——
 * 表现为「明明没改代码，测试却红了」，极难归因。
 * TLS 的代价只是每个真正调用解码的线程各留一份，实际只有一两个线程。
 */
static _Thread_local uint8_t gf_exp[512];
static _Thread_local uint8_t gf_log[256];
static _Thread_local int gf_ready = 0;

static void gf_init(void) {
    if (gf_ready) return;
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= 0x11D;
    }
    for (int i = 255; i < 512; i++) gf_exp[i] = gf_exp[i - 255];
    gf_ready = 1;
}

static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (!a || !b) return 0;
    return gf_exp[gf_log[a] + gf_log[b]];
}

static inline uint8_t gf_inv(uint8_t a) {
    return gf_exp[255 - gf_log[a]];
}

/* 多项式求值（系数最高次在前），x = alpha^power 由调用方给出 */
static uint8_t poly_eval(const uint8_t *p, int len, uint8_t x) {
    uint8_t v = 0;
    for (int i = 0; i < len; i++) v = gf_mul(v, x) ^ p[i];
    return v;
}

/*
 * RS errata 解码：block 长 n（数据 + 32 奇偶），eras 为擦除位置（下标）数组。
 * 就地纠错，成功返回 0。标准流程：
 *   擦除定位多项式 Γ -> 修正综合征 Ξ = S·Γ -> BM 求错误定位 σ ->
 *   errata Λ = σ·Γ -> Chien -> Forney（Ω = S·Λ mod x^32）
 *
 * out_fixed 可为 NULL；非 NULL 时回填本块实际改写的字节数（综合征全零即 0）。
 * 只在成功路径回填：失败时纠正量没有意义，块内容也未必被改对。
 */
/* 多项式全部采用「最高次在前」约定，与 dense_refimpl.py 的解码器一致 */
static int rs_errata_decode(uint8_t *block, int n,
                            const int *eras, int n_eras, int *out_fixed) {
    if (out_fixed) *out_fixed = 0;
    if (n_eras > RS_ERAS_MAX) return -1;
    uint8_t synd[RS_PARITY];
    int all_zero = 1;
    for (int i = 0; i < RS_PARITY; i++) {
        synd[i] = poly_eval(block, n, gf_exp[i]);
        if (synd[i]) all_zero = 0;
    }
    if (all_zero) return 0;

    /* Γ(x) = prod (xi·x + 1)，高次在前；同时保留低次在前副本做卷积 */
    uint8_t ghf[RS_PARITY + 1];
    int glen = 1;
    ghf[0] = 1;
    for (int e = 0; e < n_eras; e++) {
        uint8_t xi = gf_exp[(n - 1 - eras[e]) % 255];
        uint8_t next[RS_PARITY + 2];
        memset(next, 0, sizeof next);
        for (int i = 0; i < glen; i++) {
            next[i] ^= gf_mul(ghf[i], xi);       /* xi·x 项 */
            next[i + 1] ^= ghf[i];               /* 常数 1 项 */
        }
        glen++;
        memcpy(ghf, next, (size_t)glen);
    }

    /* 修正综合征 Ξ = S·Γ mod x^32（低次在前卷积，Γ 低次 = ghf 反转）。
       下标 < e 的低段被擦除污染，BM 只能吃 Ξ[e..31] —— 即 Forney 综合征 */
    uint8_t xs_full[RS_PARITY];
    for (int i = 0; i < RS_PARITY; i++) {
        uint8_t acc = 0;
        for (int j = 0; j <= i && j < glen; j++) {
            acc ^= gf_mul(ghf[glen - 1 - j], synd[i - j]);
        }
        xs_full[i] = acc;
    }
    const uint8_t *xs = xs_full + n_eras;

    /* BM（高次在前，逐行对照 dense_refimpl.py 的实现） */
    uint8_t err_loc[RS_PARITY + 2], old_loc[RS_PARITY + 2], tmp[RS_PARITY + 2];
    int elen = 1, olen = 1;
    err_loc[0] = 1;
    old_loc[0] = 1;
    int budget = RS_PARITY - n_eras;
    for (int i = 0; i < budget; i++) {
        old_loc[olen++] = 0;                     /* 乘 x */
        uint8_t delta = xs[i];
        for (int j = 1; j < elen; j++) {
            delta ^= gf_mul(err_loc[elen - 1 - j], xs[i - j]);
        }
        if (delta) {
            if (olen > elen) {
                for (int j = 0; j < olen; j++) tmp[j] = gf_mul(old_loc[j], delta);
                uint8_t inv = gf_inv(delta);
                for (int j = 0; j < elen; j++) old_loc[j] = gf_mul(err_loc[j], inv);
                int t = olen;
                olen = elen;
                elen = t;
                memcpy(err_loc, tmp, (size_t)elen);
                /* 交换后仍要做下面的无条件异或（对照 Python 实现） */
            }
            /* err_loc ^= delta·old_loc（old_loc 尾对齐） */
            int off = elen - olen;
            for (int j = 0; j < olen; j++) {
                err_loc[off + j] ^= gf_mul(old_loc[j], delta);
            }
        }
    }
    /* 去掉前导零 */
    int lead = 0;
    while (lead < elen - 1 && err_loc[lead] == 0) lead++;
    elen -= lead;
    memmove(err_loc, err_loc + lead, (size_t)elen);
    int n_err = elen - 1;
    if (2 * n_err > budget) return -1;

    /* Λ = σ·Γ（高次在前卷积） */
    uint8_t lam[2 * RS_PARITY + 4];
    int llen = elen + glen - 1;
    if (llen > (int)sizeof lam) return -1;
    memset(lam, 0, sizeof lam);
    for (int i = 0; i < elen; i++) {
        if (!err_loc[i]) continue;
        for (int j = 0; j < glen; j++) lam[i + j] ^= gf_mul(err_loc[i], ghf[j]);
    }
    int lam_deg = llen - 1;
    if (lam_deg > RS_PARITY) return -1;

    /* Chien：Λ(Xi^-1) = 0 的位置 */
    int pos[RS_PARITY];
    int n_pos = 0;
    for (int i = 0; i < n; i++) {
        uint8_t x = gf_exp[(255 - ((n - 1 - i) % 255)) % 255];
        if (poly_eval(lam, llen, x) == 0) {
            if (n_pos >= RS_PARITY) return -1;
            pos[n_pos++] = i;
        }
    }
    if (n_pos != lam_deg) return -1;

    /* Ω = reversed(S)·Λ 的低 32 次（高次在前取尾段） */
    uint8_t srev[RS_PARITY];
    for (int i = 0; i < RS_PARITY; i++) srev[i] = synd[RS_PARITY - 1 - i];
    uint8_t prod[3 * RS_PARITY + 4];
    int prod_len = RS_PARITY + llen - 1;
    memset(prod, 0, sizeof prod);
    for (int i = 0; i < RS_PARITY; i++) {
        if (!srev[i]) continue;
        for (int j = 0; j < llen; j++) prod[i + j] ^= gf_mul(srev[i], lam[j]);
    }
    uint8_t *omega = prod + (prod_len - RS_PARITY);

    int n_fixed = 0;
    for (int k = 0; k < n_pos; k++) {
        uint8_t xi = gf_exp[(n - 1 - pos[k]) % 255];
        uint8_t xi_inv = gf_inv(xi);
        uint8_t ov = poly_eval(omega, RS_PARITY, xi_inv);
        /* 形式导数：power = lam_deg - idx 为奇数的项，co·xi_inv^(power-1) */
        uint8_t dv = 0;
        for (int idx = 0; idx < llen; idx++) {
            int power = lam_deg - idx;
            if (power % 2 == 1 && lam[idx]) {
                int e = ((int)gf_log[xi_inv] * (power - 1)) % 255;
                dv ^= gf_mul(lam[idx], gf_exp[e]);
            }
        }
        if (!dv) return -1;
        uint8_t mag = gf_mul(gf_mul(ov, gf_inv(dv)), xi);   /* fcr=0 修正因子 */
        block[pos[k]] ^= mag;
        /* 幅值为 0 的位置（擦除处恰好本来就对）不算纠正，否则读数会虚高 */
        if (mag) n_fixed++;
    }
    /* 复验 */
    for (int i = 0; i < RS_PARITY; i++) {
        if (poly_eval(block, n, gf_exp[i])) return -1;
    }
    if (out_fixed) *out_fixed = n_fixed;
    return 0;
}

/* RS 编码（仅自检用） */
static void rs_encode(const uint8_t *data, int len, uint8_t *parity) {
    static _Thread_local uint8_t gen[RS_PARITY + 1];
    static _Thread_local int gen_ready = 0;
    gf_init();
    if (!gen_ready) {
        uint8_t tmp[RS_PARITY + 1];
        gen[0] = 1;
        int glen = 1;
        for (int d = 0; d < RS_PARITY; d++) {
            memset(tmp, 0, sizeof tmp);
            for (int i = 0; i < glen; i++) {
                tmp[i] ^= gen[i];
                tmp[i + 1] ^= gf_mul(gen[i], gf_exp[d]);
            }
            glen++;
            memcpy(gen, tmp, (size_t)glen);
        }
        gen_ready = 1;
    }
    memset(parity, 0, RS_PARITY);
    for (int i = 0; i < len; i++) {
        uint8_t factor = data[i] ^ parity[0];
        memmove(parity, parity + 1, RS_PARITY - 1);
        parity[RS_PARITY - 1] = 0;
        if (factor) {
            for (int k = 0; k < RS_PARITY; k++) {
                if (gen[k + 1]) {
                    parity[k] ^= gf_exp[(gf_log[gen[k + 1]] + gf_log[factor]) % 255];
                }
            }
        }
    }
}

int vdd_selftest_rs(int trials) {
    gf_init();
    uint32_t st = 0x12345678u;
    for (int t = 0; t < trials; t++) {
        st ^= st << 13; st ^= st >> 17; st ^= st << 5;
        int dlen = 32 + (int)(st % 192);
        uint8_t block[255];
        for (int i = 0; i < dlen; i++) {
            st ^= st << 13; st ^= st >> 17; st ^= st << 5;
            block[i] = (uint8_t)st;
        }
        rs_encode(block, dlen, block + dlen);
        int n = dlen + RS_PARITY;
        uint8_t ref[255];
        memcpy(ref, block, (size_t)n);
        /* 注入 e 个擦除 + v 个错误，e + 2v <= 32 */
        int e = (int)(st % 20);
        int v = (int)((RS_PARITY - e) / 2 > 0 ? (st >> 8) % (uint32_t)((RS_PARITY - e) / 2 + 1) : 0);
        int eras[RS_ERAS_MAX];
        uint8_t used[255];
        memset(used, 0, sizeof used);
        for (int i = 0; i < e; i++) {
            int p;
            do { st ^= st << 13; st ^= st >> 17; st ^= st << 5; p = (int)(st % (uint32_t)n); } while (used[p]);
            used[p] = 1;
            eras[i] = p;
            block[p] ^= (uint8_t)(st >> 16) | 1;
        }
        for (int i = 0; i < v; i++) {
            int p;
            do { st ^= st << 13; st ^= st >> 17; st ^= st << 5; p = (int)(st % (uint32_t)n); } while (used[p]);
            used[p] = 1;
            block[p] ^= (uint8_t)(st >> 16) | 1;
        }
        if (rs_errata_decode(block, n, eras, e, NULL) != 0) return -(t + 1);
        if (memcmp(block, ref, (size_t)n) != 0) return -(t + 1);
    }
    return 0;
}

/* ---------------------------------------------------------------- 结构 */

static int is_valid_grid(int n) {
    for (int i = 0; i < N_GRIDS; i++) if (GRID_SIZES[i] == n) return 1;
    return 0;
}

static inline int in_corner_block(int r, int c, int n) {
    return (r < CORNER && c < CORNER) || (r < CORNER && c >= n - CORNER) ||
           (r >= n - CORNER && c < CORNER) ||
           (r >= n - CORNER && c >= n - CORNER);
}

static void cal_strips(int n, int strips[4][2]) {
    strips[0][0] = CORNER;         strips[0][1] = 1;
    strips[1][0] = CORNER;         strips[1][1] = n - 1 - CAL_LEN;
    strips[2][0] = n - 1 - CORNER; strips[2][1] = 1;
    strips[3][0] = n - 1 - CORNER; strips[3][1] = n - 1 - CAL_LEN;
}

/* 自由格枚举（行优先），free_idx 容量须 >= n*n；返回自由格数 */
static int free_cells(int n, int32_t *free_idx) {
    uint8_t fn[N_MAX * N_MAX];
    memset(fn, 0, (size_t)(n * n));
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++)
            if (in_corner_block(r, c, n)) fn[r * n + c] = 1;
    for (int i = 0; i < n; i++) {
        fn[i] = 1; fn[(n - 1) * n + i] = 1;
        fn[i * n] = 1; fn[i * n + n - 1] = 1;
    }
    int strips[4][2];
    cal_strips(n, strips);
    for (int s = 0; s < 4; s++)
        for (int i = 0; i < CAL_LEN; i++)
            fn[strips[s][0] * n + strips[s][1] + i] = 1;
    int cnt = 0;
    for (int i = 0; i < n * n; i++)
        if (!fn[i]) free_idx[cnt++] = i;
    return cnt;
}

/* 只需要自由格数量时用这个，省掉 n*n 的索引表 */
static int free_cells_count(int n) {
    int fn = 0;
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++) {
            if (in_corner_block(r, c, n)) continue;
            if (r == 0 || r == n - 1 || c == 0 || c == n - 1) continue;
            fn++;
        }
    int strips[4][2];
    cal_strips(n, strips);
    /* 定标条整条都在内圈、且不与角部块重叠，直接减掉 */
    return fn - 4 * CAL_LEN;
}

int vdd_net_capacity(int n_grid) {
    return vdd_net_capacity_colors(n_grid, 8);
}

int vdd_net_capacity_colors(int n_grid, int colors) {
    if (!is_valid_grid(n_grid) || (colors != 8 && colors != 4)) return 0;
    int fr = free_cells_count(n_grid);
    int cap = (fr - HEADER_CELLS) * (colors == 4 ? 2 : 3) / 8;
    for (int p = cap; p > 0; p--) {
        int nb = (p + RS_DATA_MAX - 1) / RS_DATA_MAX;
        if (p + RS_PARITY * nb <= cap) return p;
    }
    return 0;
}

static uint8_t crc8(const uint8_t *d, int len) {
    uint8_t c = 0;
    for (int i = 0; i < len; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++)
            c = (uint8_t)((c & 0x80) ? ((c << 1) ^ 0x07) : (c << 1));
    }
    return c;
}

/* ---------------------------------------------------------------- 像素访问 */

static inline void px_rgb(const vdd_image *img, int x, int y, uint8_t out[3]) {
    const uint8_t *p = img->pixels + (size_t)y * img->stride;
    if (img->is_bgra) {
        p += (size_t)x * 4;
        out[0] = p[2]; out[1] = p[1]; out[2] = p[0];
    } else {
        p += (size_t)x * 3;
        out[0] = p[0]; out[1] = p[1]; out[2] = p[2];
    }
}

static inline int px_luma(const vdd_image *img, int x, int y) {
    uint8_t c[3];
    px_rgb(img, x, y, c);
    return (c[0] + 2 * c[1] + c[2]) >> 2;
}

/* 双线性采样 RGB（浮点坐标） */
static void sample_bilinear(const vdd_image *img, double x, double y, double out[3]) {
    int w = img->width, h = img->height;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > w - 1) x = w - 1;
    if (y > h - 1) y = h - 1;
    int x0 = (int)x, y0 = (int)y;
    int x1 = x0 + 1 < w ? x0 + 1 : x0;
    int y1 = y0 + 1 < h ? y0 + 1 : y0;
    double fx = x - x0, fy = y - y0;
    /* 热路径：直接取四个像素的指针，不逐像素走 px_rgb 的格式分支 */
    const size_t bpp = img->is_bgra ? 4 : 3;
    const int ri = img->is_bgra ? 2 : 0;     /* R 所在字节；B 在 2 - ri */
    const uint8_t *row0 = img->pixels + (size_t)y0 * img->stride;
    const uint8_t *row1 = img->pixels + (size_t)y1 * img->stride;
    const uint8_t *p00 = row0 + (size_t)x0 * bpp, *p01 = row0 + (size_t)x1 * bpp;
    const uint8_t *p10 = row1 + (size_t)x0 * bpp, *p11 = row1 + (size_t)x1 * bpp;
    const int src[3] = { ri, 1, 2 - ri };
    for (int ch = 0; ch < 3; ch++) {
        int o = src[ch];
        double a = p00[o] * (1 - fx) + p01[o] * fx;
        double b = p10[o] * (1 - fx) + p11[o] * fx;
        out[ch] = a * (1 - fy) + b * fy;
    }
}

/* ---------------------------------------------------------------- 几何 */

/* 4 点对 dst(格坐标)->src(像素)，输出 h[9]。8x8 高斯消元。 */
static int homography(const double dst[4][2], const double src[4][2], double h[9]) {
    double M[8][9];
    for (int i = 0; i < 4; i++) {
        double dx = dst[i][0], dy = dst[i][1];
        double sx = src[i][0], sy = src[i][1];
        double r0[9] = { dx, dy, 1, 0, 0, 0, -dx * sx, -dy * sx, sx };
        double r1[9] = { 0, 0, 0, dx, dy, 1, -dx * sy, -dy * sy, sy };
        memcpy(M[i * 2], r0, sizeof r0);
        memcpy(M[i * 2 + 1], r1, sizeof r1);
    }
    for (int col = 0; col < 8; col++) {
        int piv = col;
        for (int r = col + 1; r < 8; r++)
            if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
        if (fabs(M[piv][col]) < 1e-12) return -1;
        if (piv != col) {
            for (int j = 0; j < 9; j++) {
                double t = M[col][j]; M[col][j] = M[piv][j]; M[piv][j] = t;
            }
        }
        double d = M[col][col];
        for (int j = 0; j < 9; j++) M[col][j] /= d;
        for (int r = 0; r < 8; r++) {
            if (r == col || M[r][col] == 0) continue;
            double f = M[r][col];
            for (int j = 0; j < 9; j++) M[r][j] -= f * M[col][j];
        }
    }
    for (int i = 0; i < 8; i++) h[i] = M[i][8];
    h[8] = 1.0;
    return 0;
}

static inline void apply_h(const double h[9], double x, double y,
                           double *ox, double *oy) {
    double d = h[6] * x + h[7] * y + h[8];
    *ox = (h[0] * x + h[1] * y + h[2]) / d;
    *oy = (h[3] * x + h[4] * y + h[5]) / d;
}

/*
 * 符号几何：单应性 + 边缘弓曲校正场。
 *
 * 单应性只能表达平面投影，无法表达镜头径向（桶形/枕形）畸变——后者会让
 * 符号的四条边向内或向外弓起。四个角点已被定位图形钉死，形变全部体现在
 * 边中部，实测未校正时 96 格符号在中等桶形畸变下整帧解不出。
 *
 * 校正场 = 四条边的实测法向偏移剖面 + 横向形状函数：
 *     m(s) = (f_上(s) + f_下(s)) / 2        对称分量，等价于平移
 *     A(s) = (f_上(s) − f_下(s)) / 2        反对称分量，即径向弓曲
 *     dy(s,t) = m(s) + A(s) · shape(u)，    u = 1 − 2t
 *     shape(u) = β·u + (1−β)·u³             满足 shape(±1)=±1
 *
 * 横向剖面的形状是实测出来的，不是猜的：对已知桶形畸变解析求出真实形变场后，
 * 沿垂直中心线拟合得 d(u) = 2.495·u − 1.389·u³，逐点吻合，即 β≈2.26。
 * 峰值出现在 t≈0.1 而非边缘——这是任何单参数插值都表达不了的形状。
 *
 * 边缘测量只给出 shape(±1)（即 a+b），无法拆分 β，故 β 由**判决置信度**搜索确定：
 * 几何正确时采样点远离判决阈值，几何错误时大量格落在边界上、置信度塌陷。
 * 该搜索只在建锁的首帧做一次，结果随跟踪锁复用（几何在会话内是稳定的）。
 *
 * 失败尝试留档，避免重蹈：
 * 1) 线性（Coons）混合，即 β=1——边缘与中心都对，中等半径处 64% 错格。
 * 2) 纯三次 u³，即 β=0——比线性更差（57% 错格），横向衰减过快。
 * 3) 显式拟合 d = shift + α·ρ²·(g−c)——5 参数在只有边缘观测时病态，
 *    网格搜索收敛到 α≈0、中心跑到符号外的退化解，错格 49%。
 */
#define EDGE_SEGS 5
#define BETA_MIN 0.6
#define BETA_MAX 3.4
#define BETA_STEPS 15

typedef struct {
    double h[9];
    int n;
    double prof[4][EDGE_SEGS];   /* 上/下/左/右 边的法向偏移剖面，单位=格 */
    double at[EDGE_SEGS];        /* 各段中点的归一化位置 */
    double beta;                 /* 横向形状参数，1=线性 */
    double cell_px;              /* 每格像素数，决定采样点数 */
    int refined;
} vdd_geom;

/* 剖面在归一化位置 u 处的线性插值（两端外推为常数） */
static double prof_at(const vdd_geom *g, int e, double u) {
    if (u <= g->at[0]) return g->prof[e][0];
    if (u >= g->at[EDGE_SEGS - 1]) return g->prof[e][EDGE_SEGS - 1];
    for (int i = 0; i + 1 < EDGE_SEGS; i++) {
        if (u <= g->at[i + 1]) {
            double w = (u - g->at[i]) / (g->at[i + 1] - g->at[i]);
            return g->prof[e][i] * (1.0 - w) + g->prof[e][i + 1] * w;
        }
    }
    return g->prof[e][EDGE_SEGS - 1];
}

/* 横向形状：u = 1−2t，shape(±1)=±1，β=1 退化为线性 */
static inline double blend_shape(double beta, double t) {
    double u = 1.0 - 2.0 * t;
    return beta * u + (1.0 - beta) * u * u * u;
}

/* 格坐标 -> 像素坐标（含畸变校正） */
static inline void geom_map(const vdd_geom *g, double gx, double gy,
                            double *px, double *py) {
    if (!g->refined) {
        apply_h(g->h, gx, gy, px, py);
        return;
    }
    double s = gx / g->n, t = gy / g->n;
    double ft = prof_at(g, 0, s), fb = prof_at(g, 1, s);
    double fl = prof_at(g, 2, t), fr = prof_at(g, 3, t);
    double dy = (ft + fb) * 0.5 + (ft - fb) * 0.5 * blend_shape(g->beta, t);
    double dx = (fl + fr) * 0.5 + (fl - fr) * 0.5 * blend_shape(g->beta, s);
    apply_h(g->h, gx + dx, gy + dy, px, py);
}

static inline void geom_reset(vdd_geom *g) {
    g->refined = 0;
    g->beta = 1.0;
    for (int e = 0; e < 4; e++)
        for (int i = 0; i < EDGE_SEGS; i++) g->prof[e][i] = 0.0;
}

/*
 * 格内面积平均采样：格心邻域按 k×k 取样后平均。
 *
 * 摩尔纹是屏幕像素栅格与传感器栅格的拍频，属高频分量；单点采样没有任何
 * 低通，拍频会直接混叠进判决。面积平均即低通滤波。取样限制在格内侧 60%，
 * 避开相邻格渗透。k 随每格像素数自适应——像素本就不够时多点采样无意义。
 */
static void sample_cell(const vdd_image *img, const vdd_geom *g,
                        double gx, double gy, double out[3]) {
    /*
     * 格内面积平均的采样点数（k×k），封顶 3。
     *
     * 原本 cell_px >= 8 时用 4×4，直觉是「格子大就多采几个点更稳」，实测反了：
     * 采样点铺在格内侧 60% 的方框上，k 越大最外圈的点越贴近格边界，混进来的
     * 邻格串扰也越多。封到 3×3 之后一张真机 21/22 的照片变成 22/22 整帧解出，
     * 4K 诊断包里两帧从纠错失败变成全解，金图像 8/8 无一退步，采样耗时还降了
     * 约四成（真机 N=112 实测 23 ms -> 14 ms 量级）。再降到 2×2 则开始变差。
     */
    int k = 1;
    if (g->cell_px >= 2.5) k = 2;
    if (g->cell_px >= 4.0) k = 3;
    if (k == 1) {
        double px, py;
        geom_map(g, gx, gy, &px, &py);
        sample_bilinear(img, px, py, out);
        return;
    }
    /*
     * 格内 k×k 个点按格心处的局部仿射展开：格心映射一次，求出映射在格心的
     * 雅可比，其余点只做乘加。原来每个点各走一遍 geom_map（含透视除法，
     * 边缘校正启用时还有剖面插值与形状函数），每格 9 次降到 1 次（纯单应）
     * 或 3 次（边缘校正，雅可比用 0.3 格步长的前向差分）。
     * 误差是二阶项：纯单应为 h6·偏移²·每格像素，±0.3 格偏移、强透视下也在
     * 0.01 px 以下；边缘校正场在全符号尺度上才弯 1–2 格，0.3 格内的曲率
     * 贡献约 1e-4 格。回归样本上判决结果逐格不变。
     */
    double cx, cy, jxx, jxy, jyx, jyy;
    if (!g->refined) {
        const double *h = g->h;
        double X = h[0] * gx + h[1] * gy + h[2];
        double Y = h[3] * gx + h[4] * gy + h[5];
        double iw = 1.0 / (h[6] * gx + h[7] * gy + h[8]);
        cx = X * iw; cy = Y * iw;
        jxx = (h[0] - cx * h[6]) * iw; jxy = (h[1] - cx * h[7]) * iw;
        jyx = (h[3] - cy * h[6]) * iw; jyy = (h[4] - cy * h[7]) * iw;
    } else {
        double ax, ay, bx, by;
        geom_map(g, gx, gy, &cx, &cy);
        geom_map(g, gx + 0.3, gy, &ax, &ay);
        geom_map(g, gx, gy + 0.3, &bx, &by);
        jxx = (ax - cx) / 0.3; jyx = (ay - cy) / 0.3;
        jxy = (bx - cx) / 0.3; jyy = (by - cy) / 0.3;
    }
    double acc[3] = { 0, 0, 0 };
    for (int i = 0; i < k; i++) {
        double ox = -0.3 + 0.6 * (i + 0.5) / k;
        for (int j = 0; j < k; j++) {
            double oy = -0.3 + 0.6 * (j + 0.5) / k;
            double c[3];
            sample_bilinear(img, cx + jxx * ox + jxy * oy, cy + jyx * ox + jyy * oy, c);
            acc[0] += c[0]; acc[1] += c[1]; acc[2] += c[2];
        }
    }
    double inv = 1.0 / (k * k);
    out[0] = acc[0] * inv; out[1] = acc[1] * inv; out[2] = acc[2] * inv;
}

/*
 * 用边框环标定四条边的法向偏移。
 *
 * 边框环是最外一圈中角部功能块之外的格，按 (r+c) 奇偶交替黑白——沿边方向
 * 是一个周期已知的方波。把采样线沿法向平移，与该已知图案做相关，相关值最大
 * 处即真实边线位置。这把「四个角点」扩展成「四条边的实测形状」，
 * 无需改协议、无需相机内参。
 */
#define REFINE_STEPS 81         /* -2.0 … +2.0，步长 0.05 */

static void refine_edges(const vdd_image *img, vdd_geom *g) {
    const int n = g->n;
    const int lo = CORNER + 1, hi = n - CORNER - 1;
    const int span = hi - lo;
    if (span < EDGE_SEGS * 2) return;          /* 边太短，放弃校正 */

    for (int i = 0; i < EDGE_SEGS; i++) {
        int a0 = lo + span * i / EDGE_SEGS;
        int a1 = lo + span * (i + 1) / EDGE_SEGS;
        g->at[i] = ((a0 + a1) * 0.5) / n;
    }
    for (int e = 0; e < 4; e++) {
        for (int seg = 0; seg < EDGE_SEGS; seg++) {
            int a0 = lo + span * seg / EDGE_SEGS;
            int a1 = lo + span * (seg + 1) / EDGE_SEGS;
            double score[REFINE_STEPS];
            double best = -1e18;
            int best_i = REFINE_STEPS / 2;
            for (int i = 0; i < REFINE_STEPS; i++) {
                double off = -2.0 + 0.05 * i;
                double sum = 0.0;
                /* 刚性平移该段（基于原始单应性），测量不被插值权重稀释 */
                for (int a = a0; a < a1; a++) {
                    double gx, gy;
                    int r, c;
                    switch (e) {
                        case 0: gx = a + 0.5;  gy = 0.5 + off;      r = 0;     c = a; break;
                        case 1: gx = a + 0.5;  gy = n - 0.5 + off;  r = n - 1; c = a; break;
                        case 2: gx = 0.5 + off; gy = a + 0.5;       r = a;     c = 0; break;
                        default: gx = n - 0.5 + off; gy = a + 0.5;  r = a;     c = n - 1; break;
                    }
                    double px, py, rgb[3];
                    apply_h(g->h, gx, gy, &px, &py);
                    sample_bilinear(img, px, py, rgb);
                    double luma = (rgb[0] + 2 * rgb[1] + rgb[2]) / 4.0;
                    sum += ((r + c) % 2 == 0) ? -luma : luma;  /* (r+c) 偶数期望为黑 */
                }
                score[i] = sum;
                if (sum > best) { best = sum; best_i = i; }
            }
            /*
             * 取平台中点而非 argmax。格是纯色块，采样点只要落在格内分数就相同，
             * 打分曲线是一段平台不是尖峰；argmax 会落到平台边缘（贴着格边界），
             * 中点才是离两侧边界最远、最抗噪的位置。这是实测踩到的偏差来源。
             */
            double floor_v = best > 0 ? best * 0.97 : best - fabs(best) * 0.03;
            int l = best_i, r2 = best_i;
            while (l > 0 && score[l - 1] >= floor_v) l--;
            while (r2 < REFINE_STEPS - 1 && score[r2 + 1] >= floor_v) r2++;
            g->prof[e][seg] = -2.0 + 0.05 * ((l + r2) * 0.5);
        }
    }
    g->refined = 1;
}

/*
 * 边框环整体吻合度：沿全环按 (r+c) 偶=黑的期望累加带符号亮度，越大越吻合。
 * 与 refine_edges 内部的打分同一量纲，但覆盖整圈而非单段。
 */
static double ring_score(const vdd_image *img, const vdd_geom *g) {
    const int n = g->n;
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        const int cells[4][2] = { { 0, i }, { n - 1, i }, { i, 0 }, { i, n - 1 } };
        for (int k = 0; k < 4; k++) {
            int r = cells[k][0], c = cells[k][1];
            if (in_corner_block(r, c, n)) continue;
            double rgb[3];
            sample_cell(img, g, c + 0.5, r + 0.5, rgb);
            double luma = (rgb[0] + 2 * rgb[1] + rgb[2]) / 4.0;
            sum += ((r + c) % 2 == 0) ? -luma : luma;
        }
    }
    return sum;
}

/*
 * 边缘校正的自校验闸门。
 *
 * refine_edges 是逐段独立测量，段与段之间没有平滑约束，透视较强或每格像素偏少
 * 时会各自收敛到 ±2 格的搜索边界，拼出一条锯齿状的假边——真机实测有照片
 * 因此从「纯单应 448/448 全对」被打坏到 342/448。校正本是为了补镜头畸变，
 * 补出负收益就该退回去。
 *
 * 判据用整圈吻合度（校正前后各算一次）：桶形畸变的图像上校正确有增益，
 * 会被保留；打坏的则原样退回纯单应。代价是多一轮环采样，只在建锁帧发生。
 */
static void refine_edges_guarded(const vdd_image *img, vdd_geom *g) {
    double before = ring_score(img, g);
    refine_edges(img, g);
    if (!g->refined) return;
    if (ring_score(img, g) < before) g->refined = 0;
}

static const int CHAN_BIT[3] = { 4, 2, 1 };

/*
 * 定标条实测调色板 + 最近邻判决。
 *
 * 为什么不能用三通道独立阈值——真机实测的定标条读数（N=128 照片）：
 *
 *     v=2 期望 -G-  实测 (114,185, 62)
 *     v=3 期望 -GB  实测 (118,201,179)
 *
 * 绿与青这两个颜色，R 通道读出来是 114–118 而不是应有的 ~20。屏幕的绿基色
 * 在相机传感器的 R 通道上有很大响应（色彩矩阵串扰），R 通道因此是**三峰**分布：
 * 暗簇 ~20、中簇 ~115（绿/青）、亮簇 ~190。二均值聚类只能切一刀，中簇落在
 * 哪一侧全看两簇的相对样本量——同一台设备，N=96 的照片切出 R 阈值 150（中簇判暗，
 * 侥幸正确），N=128 的切出 107（中簇判亮，绿格青格的 R 位整类错掉，2/8 的颜色报废）。
 * 表现就是几何全对、边框环 448/448 全对，RS 却 0/22：边框环只有纯黑纯白，
 * 根本碰不到中簇。
 *
 * 改按实测调色板做三维最近邻即可，串扰是什么形状都无所谓——它本来就被测进去了。
 * 四条定标条正好在四角，按位置双线性插值还顺带补偿了光照梯度与白平衡不均。
 * 实测六张真机照片全部整帧解出，其中一张纠错次数为 0。
 *
 * 定标条失效时（低像素密度下相邻格互相污染，颜色簇挤到一起）退回阈值判决，
 * 判据是八色两两最小距离。
 */
#define PAL_MIN_SEP 40.0        /* 八色两两最小欧氏距离，低于此判定调色板不可用 */
#define PAL_ERASE 0.10          /* 归一化裕度低于此标记为擦除 */

typedef struct {
    double c[4][8][3];          /* 角序 TL,TR,BL,BR，各 8 色 RGB */
    int valid;
    int valid4;                 /* 只看 4 色档用到的四色是否分得开 */
} vdd_palette;

/* 采四条定标条建表；八色分不开则 valid=0，选定四色分不开则 valid4=0 */
static void build_palette(const vdd_image *img, const vdd_geom *g, vdd_palette *pal) {
    const int n = g->n;
    int strips[4][2];
    cal_strips(n, strips);
    for (int s = 0; s < 4; s++) {
        for (int v = 0; v < CAL_LEN; v++) {
            double p[3];
            sample_cell(img, g, strips[s][1] + v + 0.5, strips[s][0] + 0.5, p);
            for (int k = 0; k < 3; k++) pal->c[s][v][k] = p[k];
        }
    }
    pal->valid = 1;
    for (int s = 0; s < 4 && pal->valid; s++) {
        for (int u = 0; u < 8; u++) {
            for (int v = u + 1; v < 8; v++) {
                double d = 0;
                for (int k = 0; k < 3; k++) {
                    double e = pal->c[s][u][k] - pal->c[s][v][k];
                    d += e * e;
                }
                if (d < PAL_MIN_SEP * PAL_MIN_SEP) { pal->valid = 0; break; }
            }
        }
    }
    /* 定标条照旧测全八色，4 色档只取其中四色，间距门限同上 */
    pal->valid4 = 1;
    for (int s = 0; s < 4 && pal->valid4; s++)
        for (int u = 0; u < 4 && pal->valid4; u++)
            for (int v = u + 1; v < 4; v++) {
                double d = 0;
                for (int k = 0; k < 3; k++) {
                    double e = pal->c[s][PALETTE4[u]][k] - pal->c[s][PALETTE4[v]][k];
                    d += e * e;
                }
                if (d < PAL_MIN_SEP * PAL_MIN_SEP) { pal->valid4 = 0; break; }
            }
}

/* 位置 (s,t)（归一化 0..1）处第 v 色的插值参考值 */
static inline void palette_at(const vdd_palette *pal, double s, double t,
                              int v, double out[3]) {
    double w[4] = { (1 - s) * (1 - t), s * (1 - t), (1 - s) * t, s * t };
    for (int k = 0; k < 3; k++)
        out[k] = w[0] * pal->c[0][v][k] + w[1] * pal->c[1][v][k]
               + w[2] * pal->c[2][v][k] + w[3] * pal->c[3][v][k];
}

/* 在 vs[0..nv) 这几色里做最近邻，返回下标；margin 为归一化裕度（0=两色等距，1=完全确定）*/
static int classify_cell(const vdd_palette *pal, double s, double t,
                         const int *vs, int nv, const double p[3], double *margin) {
    double best = 1e30, second = 1e30;
    int bi = 0;
    for (int v = 0; v < nv; v++) {
        double ref[3], d = 0;
        palette_at(pal, s, t, vs[v], ref);
        for (int k = 0; k < 3; k++) { double e = p[k] - ref[k]; d += e * e; }
        if (d < best) { second = best; best = d; bi = v; }
        else if (d < second) second = d;
    }
    if (margin) {
        double b = sqrt(best), sc = sqrt(second);
        *margin = (sc - b) / (sc + b + 1e-9);
    }
    return bi;
}

/* 头部格只写 0(黑) 或 7(白)，只在这两色间比，比全八色判决更抗噪 */
static int classify_dark(const vdd_palette *pal, double s, double t,
                         const double p[3]) {
    double d0 = 0, d7 = 0, r0[3], r7[3];
    palette_at(pal, s, t, 0, r0);
    palette_at(pal, s, t, 7, r7);
    for (int k = 0; k < 3; k++) {
        double a = p[k] - r0[k], b = p[k] - r7[k];
        d0 += a * a; d7 += b * b;
    }
    return d0 < d7;
}

/*
 * 逐行预计算的调色板。palette_at 是位置的双线性函数，同一行内 t 固定，
 * 可写成 ref(s) = L + s·(R − L)，每格每色只剩 3 次乘加，
 * 不必每格对 8 色各算一遍四角权重（原来每格 8 次 palette_at，偏移校正再 8 次）。
 */
typedef struct {
    double a[8][3], b[8][3];          /* 下标是判决下标（8 色即格值，4 色即 2 bit 值） */
    /* 在用各色在该行上的一阶、二阶矩（分区光照模型用）：
       均值 = ma + s·mb，均方 = qaa + 2s·qab + s²·qbb */
    double ma[3], mb[3], qaa[3], qab[3], qbb[3];
} pal_row;

/* vs[0..nv) 为在用的颜色：8 色档全部八色，4 色档只有选定四色——
   矩匹配的前提是「数据格在这几色上等概率」，矩只能按在用的颜色算 */
static void pal_row_init(const vdd_palette *pal, double t, const int *vs, int nv,
                         pal_row *pr) {
    for (int k = 0; k < 3; k++) {
        pr->ma[k] = pr->mb[k] = pr->qaa[k] = pr->qab[k] = pr->qbb[k] = 0;
        for (int i = 0; i < nv; i++) {
            int v = vs[i];
            double l = (1 - t) * pal->c[0][v][k] + t * pal->c[2][v][k];
            double r = (1 - t) * pal->c[1][v][k] + t * pal->c[3][v][k];
            double a = l, b = r - l;
            pr->a[i][k] = a;
            pr->b[i][k] = b;
            pr->ma[k] += a / nv; pr->mb[k] += b / nv;
            pr->qaa[k] += a * a / nv; pr->qab[k] += a * b / nv; pr->qbb[k] += b * b / nv;
        }
    }
}

/* 对给定的 nv 色参考值做最近邻，返回下标，margin 定义同 classify_cell */
static int classify_ref(const double ref[8][3], int nv, const double p[3], double *margin) {
    double best = 1e30, second = 1e30;
    int bi = 0;
    for (int v = 0; v < nv; v++) {
        double e0 = p[0] - ref[v][0], e1 = p[1] - ref[v][1], e2 = p[2] - ref[v][2];
        double d = e0 * e0 + e1 * e1 + e2 * e2;
        if (d < best) { second = best; best = d; bi = v; }
        else if (d < second) second = d;
    }
    double b = sqrt(best), sc = sqrt(second);
    *margin = (sc - b) / (sc + b + 1e-9);
    return bi;
}

/*
 * 分区光照模型。
 *
 * 四条定标条只在四角，双线性插值只能表达「从一角到另一角的线性渐变」，
 * 表达不了屏幕中部的反光斑、背光不均这类局部亮度变化——用户反馈的头号问题
 * 正是环境光干扰。这几类干扰在每个通道上都近似是仿射的：
 *     反光（向白洗）  obs = (1−k)·I + 255·k
 *     暗斑（增益降）  obs = g·I
 *     偏色            每通道各自的 g
 * 因此每个区每个通道估一组增益 a 与偏置 b，参考色取 a·P + b（P 为定标条插值）。
 *
 * 估计不依赖第一遍的判决结果（矩匹配）：载荷是 RS 编码后的字节流，八色近似
 * 等概率，一个区里数据格的均值与方差应当分别等于同位置调色板八色的均值与方差
 * 经过仿射后的值，于是 a = sqrt(var_obs / var_pal)，b = mean_obs − a·mean_pal。
 * 这是原有整体偏移校正（a=1、只估 b、全符号一个值）的推广。
 *
 * 先试过「按第一遍判决结果逐色重算簇心」：反光区里第一遍本身就大面积判错，
 * 用错的标签估出的簇心只会把错误固化，合成反光样本上几乎没有收益，
 * 而真机 N=112 诊断包反而少过了几个块。矩匹配没有这个自我强化的问题。
 *
 * 参数定义在区中心，区间按双线性插值——README 记录过「硬选最近角」在分界线上
 * 突变、实测比插值差，这里不重蹈。数据格太少的区（角部、定标条附近）退回
 * 整体偏移，即 a=1、b=bias。
 */
#define ZONE_MAX 4
#ifndef ZONE_MIN_CELLS
#define ZONE_MIN_CELLS 64       /* 区内数据格少于此数则退回整体偏移 */
#endif

typedef struct {
    int z;                                  /* 每边分区数 */
    double d[ZONE_MAX][ZONE_MAX][2][3];     /* [区行][区列] 中心处的 {增益, 偏置} × 三通道 */
} zone_delta;

/* 归一化坐标 x 在区中心之间的插值位置：下标 i0、i1 与权重 w（两端外推为常数） */
static inline void zone_lerp(int z, double x, int *i0, int *i1, double *w) {
    double f = x * z - 0.5;
    int a = (int)floor(f);
    if (a < 0) { *i0 = *i1 = 0; *w = 0; return; }
    if (a >= z - 1) { *i0 = *i1 = z - 1; *w = 0; return; }
    *i0 = a; *i1 = a + 1; *w = f - a;
}

/* 行 t 上每个区列中心处的增益与偏置（沿 t 插好），每格再沿 s 插一次 */
static void zone_row(const zone_delta *zd, double t, double out[ZONE_MAX][2][3]) {
    int r0, r1;
    double w;
    zone_lerp(zd->z, t, &r0, &r1, &w);
    for (int zc = 0; zc < zd->z; zc++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 3; k++)
                out[zc][j][k] = zd->d[r0][zc][j][k] * (1 - w) + zd->d[r1][zc][j][k] * w;
}

/*
 * 估计每通道判决阈值与半间隔。
 *
 * 主估计量用**全符号统计**而非定标条：编码块载荷是喷泉码异或的结果，
 * 每个通道天然接近 50/50 分布，对亚采样格值做二均值聚类即可稳定分出
 * 高低两簇。定标条只有 32 格、且是 0..7 快速变化的序列，在低像素密度下
 * 相邻格被光学模糊互相污染，on/off 均值双双向中间塌陷——真机首测
 * 「定标条对比度不足」就是这么来的（128 格符号在 720p 下仅约 5.5 px/格）。
 *
 * 定标条退化为交叉校验：它给出的间隔明显更大时说明全局估计被污染，
 * 取两者中更可信的一个。
 */
static int calib_thresholds(const vdd_image *img, const vdd_geom *g,
                            double thr[3], double halfgap[3]) {
    const int n = g->n;
    /* 亚采样：步长让样本数落在 600–1500 之间，兼顾稳健与开销 */
    int step = n / 26;
    if (step < 1) step = 1;
    double lo[3], hi[3];
    for (int ch = 0; ch < 3; ch++) { lo[ch] = 1e9; hi[ch] = -1e9; }

    float buf[CALIB_MAX_SAMPLES * 3];
    int cnt = 0;
    for (int r = 1; r < n - 1 && cnt * 3 + 3 <= (int)(sizeof buf / sizeof buf[0]); r += step) {
        for (int c = 1; c < n - 1 && cnt * 3 + 3 <= (int)(sizeof buf / sizeof buf[0]); c += step) {
            double p[3];
            sample_cell(img, g, c + 0.5, r + 0.5, p);
            for (int ch = 0; ch < 3; ch++) {
                float v = (float)p[ch];
                buf[cnt * 3 + ch] = v;
                if (v < lo[ch]) lo[ch] = v;
                if (v > hi[ch]) hi[ch] = v;
            }
            cnt++;
        }
    }
    if (cnt < 32) return -1;

    for (int ch = 0; ch < 3; ch++) {
        /* 二均值（Lloyd）聚类，初值取极值，迭代到稳定 */
        double a = lo[ch], b = hi[ch];
        if (b - a < 1.0) return -1;
        for (int it = 0; it < 12; it++) {
            double sa = 0, sb = 0;
            int na = 0, nb = 0;
            double mid = (a + b) * 0.5;
            for (int i = 0; i < cnt; i++) {
                double v = buf[i * 3 + ch];
                if (v < mid) { sa += v; na++; } else { sb += v; nb++; }
            }
            if (!na || !nb) return -1;
            double na2 = sa / na, nb2 = sb / nb;
            if (fabs(na2 - a) < 0.01 && fabs(nb2 - b) < 0.01) { a = na2; b = nb2; break; }
            a = na2; b = nb2;
        }
        thr[ch] = (a + b) * 0.5;
        halfgap[ch] = (b - a) * 0.5;
    }

    /* 定标条交叉校验：其间隔更大则说明全局估计被污染，改用定标条 */
    int strips[4][2];
    cal_strips(n, strips);
    double on[3] = { 0, 0, 0 }, off[3] = { 0, 0, 0 };
    for (int s = 0; s < 4; s++) {
        for (int i = 0; i < CAL_LEN; i++) {
            double c[3];
            sample_cell(img, g, strips[s][1] + i + 0.5, strips[s][0] + 0.5, c);
            for (int ch = 0; ch < 3; ch++) {
                if (i & CHAN_BIT[ch]) on[ch] += c[ch];
                else off[ch] += c[ch];
            }
        }
    }
    for (int ch = 0; ch < 3; ch++) {
        double con = on[ch] / 16.0, coff = off[ch] / 16.0;
        double chalf = (con - coff) * 0.5;
        if (chalf > halfgap[ch]) {
            thr[ch] = (con + coff) * 0.5;
            halfgap[ch] = chalf;
        }
        /* 阈值门限只挡「完全没有对比度」，真正的把关是头部 CRC */
        if (halfgap[ch] < 4.0) return -1;
    }
    return 0;
}

/*
 * 几何质量评分：抽样格的判决置信度均值（到阈值的归一化距离，取三通道最小）。
 * 几何正确时采样点落在格心、远离阈值；几何错误时大量格落在边界上，
 * 三通道读数都贴近阈值，均值显著塌陷。用它给 β 定标，无需 RS 解码。
 */
static double geom_confidence(const vdd_image *img, const vdd_geom *g) {
    double thr[3], halfgap[3];
    if (calib_thresholds(img, g, thr, halfgap) != 0) return -1e9;
    const int n = g->n;
    double sum = 0.0;
    int cnt = 0;
    for (int r = 1; r < n - 1; r += 3) {
        for (int c = 1; c < n - 1; c += 3) {
            double p[3];
            sample_cell(img, g, c + 0.5, r + 0.5, p);
            double conf = 1e9;
            for (int ch = 0; ch < 3; ch++) {
                double a = fabs(p[ch] - thr[ch]) / halfgap[ch];
                if (a < conf) conf = a;
            }
            if (conf > 2.0) conf = 2.0;      /* 饱和，避免个别高对比格主导 */
            sum += conf;
            cnt++;
        }
    }
    return cnt ? sum / cnt : -1e9;
}

/*
 * 用置信度搜索横向形状参数 β。仅在弓曲显著时才搜索——无畸变时
 * 反对称分量 A≈0，β 无影响，搜索纯属浪费。
 */
static void refine_beta(const vdd_image *img, vdd_geom *g) {
    double bow = 0.0;
    for (int i = 0; i < EDGE_SEGS; i++) {
        bow += fabs(g->prof[0][i] - g->prof[1][i]);
        bow += fabs(g->prof[2][i] - g->prof[3][i]);
    }
    bow /= (2 * EDGE_SEGS);
    if (bow < 0.35) { g->beta = 1.0; return; }   /* 弓曲可忽略 */

    double best_beta = 1.0, best_score = -1e18;
    for (int i = 0; i < BETA_STEPS; i++) {
        g->beta = BETA_MIN + (BETA_MAX - BETA_MIN) * i / (BETA_STEPS - 1);
        double sc = geom_confidence(img, g);
        if (sc > best_score) { best_score = sc; best_beta = g->beta; }
    }
    g->beta = best_beta;
}

/* ---------------------------------------------------------------- 定位 */

typedef struct { double x, y, cnt, u; } group_t;

/*
 * 局部自适应阈值（分块），思路同 ZXing 的 HybridBinarizer。
 *
 * 全局阈值在屏摄场景下不可靠：暗环境里的一块亮屏会把全局均值拉偏，
 * 二值化随之失准，定位图形的游程比例就对不上了。真机首测「定位不准 →
 * 定标条采到随机数据格 → 报对比度不足」这条链路的源头就在这里。
 *
 * 每块取动态范围足够时的中值；范围过小（块内近似纯色）的块从邻块继承，
 * 避免把噪声放大成纹理。
 */
#define LT_BLOCK 32
#define LT_MAXW 128
#define LT_MAXH 128

typedef struct {
    int bw, bh;
    uint8_t thr[LT_MAXW * LT_MAXH];
    int16_t raw[LT_MAXW * LT_MAXH];     /* 建表用的临时块均值，随结构体一起分配 */
} local_thresh;

/*
 * 局部阈值表。roi 非空时只统计与该矩形相交的块，其余块按「纯色块」走继承路径。
 *
 * 跟踪态下符号的位置是已知的，没有理由为了四个小窗的二值化去扫整幅图——
 * 4K 全图是 207 万次采样（本身已经隔行隔列），而四个跟踪窗加起来才十万像素。
 * 分块索引保持不变，所以 px_dark 一个字都不用改。
 */
static void lt_build_roi(const vdd_image *img, local_thresh *lt,
                         const int (*wins)[4], int n_wins) {
    int w = img->width, h = img->height;
    lt->bw = (w + LT_BLOCK - 1) / LT_BLOCK;
    lt->bh = (h + LT_BLOCK - 1) / LT_BLOCK;
    if (lt->bw > LT_MAXW) lt->bw = LT_MAXW;
    if (lt->bh > LT_MAXH) lt->bh = LT_MAXH;

    int16_t *raw = lt->raw;
    long total = 0;
    int tcnt = 0;
    for (int by = 0; by < lt->bh; by++) {
        for (int bx = 0; bx < lt->bw; bx++) {
            int x0 = bx * w / lt->bw, x1 = (bx + 1) * w / lt->bw;
            int y0 = by * h / lt->bh, y1 = (by + 1) * h / lt->bh;
            if (n_wins > 0) {
                int hit = 0;
                for (int i = 0; i < n_wins && !hit; i++)
                    hit = !(x1 <= wins[i][0] || x0 >= wins[i][2] ||
                            y1 <= wins[i][1] || y0 >= wins[i][3]);
                if (!hit) {
                    raw[by * lt->bw + bx] = -1;  /* 窗外，交给继承 */
                    continue;
                }
            }
            int mn = 255, mx = 0;
            for (int y = y0; y < y1; y += 2) {
                for (int x = x0; x < x1; x += 2) {
                    int v = px_luma(img, x, y);
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                }
            }
            /* 动态范围够大才本块定阈；否则标记为继承 */
            raw[by * lt->bw + bx] = (mx - mn >= 24) ? (int16_t)((mn + mx) / 2) : -1;
            if (raw[by * lt->bw + bx] >= 0) {
                total += raw[by * lt->bw + bx];
                tcnt++;
            }
        }
    }
    int fallback = tcnt ? (int)(total / tcnt) : 128;

    /* 纯色块从 3x3 邻域的有效块继承，仍无则用全局回退值 */
    for (int by = 0; by < lt->bh; by++) {
        for (int bx = 0; bx < lt->bw; bx++) {
            int idx = by * lt->bw + bx;
            if (raw[idx] >= 0) continue;
            long s = 0;
            int c = 0;
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    int yy = by + dy, xx = bx + dx;
                    if (yy < 0 || yy >= lt->bh || xx < 0 || xx >= lt->bw) continue;
                    if (raw[yy * lt->bw + xx] < 0) continue;
                    s += raw[yy * lt->bw + xx];
                    c++;
                }
            }
            raw[idx] = (int16_t)(c ? s / c : fallback);
        }
    }
    /* 3x3 平滑，消除块边界的阶跃 */
    for (int by = 0; by < lt->bh; by++) {
        for (int bx = 0; bx < lt->bw; bx++) {
            long s = 0;
            int c = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int yy = by + dy, xx = bx + dx;
                    if (yy < 0 || yy >= lt->bh || xx < 0 || xx >= lt->bw) continue;
                    s += raw[yy * lt->bw + xx];
                    c++;
                }
            }
            lt->thr[by * lt->bw + bx] = (uint8_t)(s / c);
        }
    }
}

static inline int lt_at(const local_thresh *lt, const vdd_image *img, int x, int y) {
    int bx = x * lt->bw / img->width;
    int by = y * lt->bh / img->height;
    if (bx >= lt->bw) bx = lt->bw - 1;
    if (by >= lt->bh) by = lt->bh - 1;
    return lt->thr[by * lt->bw + bx];
}

static inline int px_dark(const vdd_image *img, const local_thresh *lt, int x, int y) {
    return px_luma(img, x, y) < lt_at(lt, img, x, y);
}

/* 竖向验证 1:1:3:1:1，y 在黑芯内。 */
static int verify_v(const vdd_image *img, const local_thresh *lt,
                    int x, int y, double u) {
    int h = img->height;
    int lim = (int)(u * 5) + 2;
#define dark(yy) px_dark(img, lt, x, (yy))
    int y0 = y, steps = 0;
    while (y0 > 0 && dark(y0 - 1) && steps < lim) { y0--; steps++; }
    steps = 0;
    while (y0 > 0 && !dark(y0 - 1) && steps < lim) { y0--; steps++; }
    steps = 0;
    while (y0 > 0 && dark(y0 - 1) && steps < lim) { y0--; steps++; }
    int runs[5];
    int nr = 0, ln = 0, cur = 1;
    int yy = y0;
    while (yy < h && nr < 5) {
        int d = dark(yy);
        if (d == cur) ln++;
        else { runs[nr++] = ln; cur = d; ln = 1; }
        yy++;
    }
    if (nr < 5) {
        if (nr == 4) runs[nr++] = ln;
        else return 0;
    }
    double uu = (runs[0] + runs[1] + runs[2] + runs[3] + runs[4]) / 7.0;
    if (uu <= 0) return 0;
    static const int fac[5] = { 1, 1, 3, 1, 1 };
    for (int k = 0; k < 5; k++)
        if (fabs(runs[k] - uu * fac[k]) >= uu) return 0;
    int core_top = y0 + runs[0] + runs[1];
    return core_top <= y && y < core_top + runs[2];
}

/* 在给定行窗口内游程扫描，把候选并入分组 */
static void scan_rows(const vdd_image *img, const local_thresh *lt,
                      int y_from, int y_to, int x_from, int x_to,
                      group_t *groups, int *n_groups) {
    if (y_from < 0) y_from = 0;
    if (y_to > img->height) y_to = img->height;
    if (x_from < 0) x_from = 0;
    if (x_to > img->width) x_to = img->width;
    /*
     * 行步长固定为 2，别按图像高度放大。试过 4K 用步长 4（环有 90 px 高，
     * 采样次数看似绰绰有余），实测把两张原本 22/22 整帧解出的 12MP 实拍
     * 打成 0/22：定位图形的中心是按命中行的质心算的，行少一半质心就粗一半，
     * 角点一偏，后面全盘皆输。省下的那点扫描时间远不抵这个代价。
     */
    for (int y = y_from; y < y_to; y += 2) {
        int run_len[64];
        int run_x[64];
        int nr = 0;
        int prev = px_dark(img, lt, x_from, y);
        int ln = 1, sx = x_from;
        for (int x = x_from + 1; x <= x_to; x++) {
            int cur = (x < x_to) ? px_dark(img, lt, x, y) : !prev;
            if (x < x_to && cur == prev) { ln++; continue; }
            /* 滑窗保留最近 6 个游程 */
            if (nr == 64) {
                memmove(run_len, run_len + 32, 32 * sizeof(int));
                memmove(run_x, run_x + 32, 32 * sizeof(int));
                nr = 32;
            }
            run_len[nr] = ln;
            run_x[nr] = sx;
            nr++;
            /* prev 为暗、且窗口里有 5 段以 prev 结尾：检查 暗白暗白暗 */
            if (nr >= 5 && !prev) {
                /* 刚结束的是白段？不。游程记录的是刚结束的段：值 prev */
            }
            if (nr >= 5 && prev) {
                int i0 = nr - 5;
                /* 段值：prev 是刚结束段的值；从末往前交替，末段是暗 */
                double u = (run_len[i0] + run_len[i0 + 1] + run_len[i0 + 2] +
                            run_len[i0 + 3] + run_len[i0 + 4]) / 7.0;
                if (u < MIN_MODULE_PX) { prev = cur; ln = 1; sx = x; continue; }
                static const int fac[5] = { 1, 1, 3, 1, 1 };
                int okp = 1;
                for (int k = 0; k < 5; k++)
                    if (fabs(run_len[i0 + k] - u * fac[k]) >= u * 0.75) { okp = 0; break; }
                if (okp) {
                    double cx = run_x[i0] + (run_len[i0] + run_len[i0 + 1] +
                        run_len[i0 + 2] + run_len[i0 + 3] + run_len[i0 + 4]) / 2.0;
                    if (verify_v(img, lt, (int)cx, y, u)) {
                        int gi = -1;
                        for (int g = 0; g < *n_groups; g++) {
                            if (fabs(groups[g].x / groups[g].cnt - cx) < u * 4 &&
                                fabs(groups[g].y / groups[g].cnt - y) < u * 4) { gi = g; break; }
                        }
                        if (gi < 0) {
                            if (*n_groups < MAX_GROUP) {
                                gi = (*n_groups)++;
                            } else {
                                /* 满了就淘汰票数最少的一组，不静默丢弃新候选 */
                                int weak = 0;
                                for (int q = 1; q < *n_groups; q++)
                                    if (groups[q].cnt < groups[weak].cnt) weak = q;
                                gi = weak;
                            }
                            groups[gi].x = groups[gi].y = groups[gi].cnt = 0;
                            groups[gi].u = 0;
                        }
                        if (gi >= 0) {
                            groups[gi].x += cx;
                            groups[gi].y += y;
                            groups[gi].u += u;
                            groups[gi].cnt += 1;
                        }
                    }
                }
            }
            prev = cur;
            ln = 1;
            sx = x;
        }
    }
}

/* 找四个定位图形中心；lock 有效时只搜四角小窗。成功返回 0。 */
static int find_finders(const vdd_image *img, const vdd_lock *lock,
                        double corners[4][2]) {
    int w = img->width, h = img->height;
    /* 局部自适应阈值：全局阈值在暗环境拍亮屏时会被背景拉偏。
       约 48 KB，放堆上而非 static——static 会被并行调用的两个线程互相踩 */
    local_thresh *lt = (local_thresh *)malloc(sizeof(local_thresh));
    if (!lt) return -1;
    /*
     * 阈值表只在真正会被扫描的地方建。跟踪态下 scan_rows 只跑四角的窗，
     * 阈值表却按整幅图算，4K 全图是 207 万次采样（本身已隔行隔列），而四个
     * 大窗加起来才几十万像素。按大窗取（不是小窗），两档重试才都有阈值可用。
     */
    int wins[4][4];
    int n_wins = 0;
    if (lock && lock->valid) {
        int m = (w > h ? w : h) / 10 + 32;
        for (int k = 0; k < 4; k++) {
            int cx = (int)lock->corners[k][0], cy = (int)lock->corners[k][1];
            wins[k][0] = cx - m; wins[k][1] = cy - m;
            wins[k][2] = cx + m; wins[k][3] = cy + m;
        }
        n_wins = 4;
    }
    lt_build_roi(img, lt, wins, n_wins);

    group_t groups[MAX_GROUP];
    int n_groups = 0;
    if (lock && lock->valid) {
        /*
         * 跟踪窗先小后大。
         *
         * 原本只有一档 max(w,h)/10 + 16——这是**图像尺寸的固定比例**，1080p 下
         * 是 208 px 尚可，4K 下就成了 400 px：四个 800×800 的窗合计 2.56 M 像素，
         * 占满帧的 31%。真机遥测里跟踪态的定位耗时中位数因此仍有 20.7 ms，
         * 「跟踪」几乎没省下什么，与全图搜索只差三倍。
         *
         * 但窗口不能一味调小：手一晃符号就移出小窗，丢锁的代价是一次全图搜索，
         * 比省下的多得多。所以改成两档：先试小窗，找不齐四个再用原来的大窗，
         * 都不行才丢锁回全图。命中时省 90% 以上的扫描量，落空时只多付一次
         * 小窗扫描（本身就便宜），准确性一个字节都不让。
         */
        static const int WIN_DIV[2] = { 60, 10 };
        /* wide=1 时直接从大窗开始：调用方已经知道小窗那套角点解不出来 */
        for (int pass = lock->wide ? 1 : 0; pass < 2 && n_groups < 4; pass++) {
            n_groups = 0;
            int win = (w > h ? w : h) / WIN_DIV[pass] + 16;
            for (int k = 0; k < 4; k++) {
                int cx = (int)lock->corners[k][0], cy = (int)lock->corners[k][1];
                scan_rows(img, lt, cy - win, cy + win, cx - win, cx + win,
                          groups, &n_groups);
            }
        }
    } else {
        scan_rows(img, lt, 0, h, 0, w, groups, &n_groups);
    }
    if (n_groups < 4) { free(lt); return -1; }

    /*
     * 从候选里挑出真正构成正方形的四个。
     *
     * 早期版本按「离图像四角最近」认领，在真机上被浏览器界面的图标骗走：
     * 实测检出的四点是 上下边 2955px、左右边仅 233px 的极扁矩形，
     * 符号根本没被找到。符号不填满画面时，干扰项比真定位图形更靠近图像角点。
     *
     * 改为几何约束筛选：四点须四边近似相等、两对角线近似相等，且
     * 边长与模块尺寸之比落在合法档位范围（N−7 ∈ [57,121]，放宽到 [45,145]）。
     * 这三条同时成立的假阳性组合极为罕见。
     */
    typedef struct { double x, y, u, cnt; } cand_t;
    cand_t cd[MAX_GROUP];
    int nc = 0;
    double top = 0;
    for (int g = 0; g < n_groups; g++)
        if (groups[g].cnt > top) top = groups[g].cnt;
    for (int g = 0; g < n_groups; g++) {
        if (groups[g].cnt * 4 < top) continue;       /* 票数过低直接丢 */
        cd[nc].x = groups[g].x / groups[g].cnt;
        cd[nc].y = groups[g].y / groups[g].cnt;
        cd[nc].u = groups[g].u / groups[g].cnt;
        cd[nc].cnt = groups[g].cnt;
        nc++;
    }
    if (nc < 4) { free(lt); return -1; }
    /* 组合数按票数截断，避免候选过多时爆炸 */
    if (nc > 24) {
        for (int i = 0; i < nc; i++)
            for (int j = i + 1; j < nc; j++)
                if (cd[j].cnt > cd[i].cnt) { cand_t t = cd[i]; cd[i] = cd[j]; cd[j] = t; }
        nc = 24;
    }

    int bi[4] = { -1, -1, -1, -1 };
    double best_score = -1e30;
    int idx[4];
    for (idx[0] = 0; idx[0] < nc; idx[0]++)
    for (idx[1] = idx[0] + 1; idx[1] < nc; idx[1]++)
    for (idx[2] = idx[1] + 1; idx[2] < nc; idx[2]++)
    for (idx[3] = idx[2] + 1; idx[3] < nc; idx[3]++) {
        double px[4], py[4], pu = 0, umin = 1e30, umax = 0;
        for (int k = 0; k < 4; k++) {
            px[k] = cd[idx[k]].x; py[k] = cd[idx[k]].y;
            double uu = cd[idx[k]].u;
            pu += uu;
            if (uu < umin) umin = uu;
            if (uu > umax) umax = uu;
        }
        pu /= 4;
        if (umin <= 0 || umax / umin > 1.8) continue;   /* 模块尺寸须一致 */

        /* 按绕质心的角度排成凸四边形 */
        double cxm = (px[0] + px[1] + px[2] + px[3]) / 4;
        double cym = (py[0] + py[1] + py[2] + py[3]) / 4;
        int ord[4] = { 0, 1, 2, 3 };
        double ang[4];
        for (int k = 0; k < 4; k++) ang[k] = atan2(py[k] - cym, px[k] - cxm);
        for (int a = 0; a < 4; a++)
            for (int b = a + 1; b < 4; b++)
                if (ang[ord[b]] < ang[ord[a]]) { int t = ord[a]; ord[a] = ord[b]; ord[b] = t; }

        double side[4], smin = 1e30, smax = 0;
        for (int k = 0; k < 4; k++) {
            int p = ord[k], q = ord[(k + 1) & 3];
            side[k] = hypot(px[p] - px[q], py[p] - py[q]);
            if (side[k] < smin) smin = side[k];
            if (side[k] > smax) smax = side[k];
        }
        if (smin < 1 || smax / smin > 1.35) continue;   /* 四边近似相等 */

        double d0 = hypot(px[ord[0]] - px[ord[2]], py[ord[0]] - py[ord[2]]);
        double d1 = hypot(px[ord[1]] - px[ord[3]], py[ord[1]] - py[ord[3]]);
        if (d0 < 1 || d1 < 1) continue;
        double dr = d0 > d1 ? d0 / d1 : d1 / d0;
        if (dr > 1.25) continue;                        /* 对角线近似相等 */

        /* 边长 / 模块尺寸 = N−7，须落在合法档位范围 */
        double cells = smax / pu;
        if (cells < 45 || cells > 145) continue;

        /* 合法者中取面积最大的：真符号通常远大于界面里的干扰图标 */
        double score = smin * smin - (smax / smin - 1) * 1e4 - (dr - 1) * 1e4;
        if (score > best_score) {
            best_score = score;
            for (int k = 0; k < 4; k++) bi[k] = idx[k];
        }
    }
    if (bi[0] < 0) { free(lt); return -1; }

    /* 按位置指派四角：TL=min(x+y) BR=max(x+y) TR=max(x−y) BL=min(x−y) */
    int tl = 0, br = 0, tr = 0, bl = 0;
    for (int k = 1; k < 4; k++) {
        double sx = cd[bi[k]].x + cd[bi[k]].y, dx2 = cd[bi[k]].x - cd[bi[k]].y;
        if (sx < cd[bi[tl]].x + cd[bi[tl]].y) tl = k;
        if (sx > cd[bi[br]].x + cd[bi[br]].y) br = k;
        if (dx2 > cd[bi[tr]].x - cd[bi[tr]].y) tr = k;
        if (dx2 < cd[bi[bl]].x - cd[bi[bl]].y) bl = k;
    }
    if (tl == br || tr == bl) { free(lt); return -1; }
    int order[4] = { tl, tr, bl, br };
    for (int k = 0; k < 4; k++) {
        corners[k][0] = cd[bi[order[k]]].x;
        corners[k][1] = cd[bi[order[k]]].y;
    }
    free(lt);
    return 0;
}

/* ---------------------------------------------------------------- 解码 */

/*
 * 符号朝向。
 *
 * find_finders 按**图像空间**位置指派四角（TL=min(x+y) 等），而四个定位图形
 * 长得一模一样，符号本身没有任何朝向标记——横向拍摄、设备旋转、EXIF 方向
 * 都会让符号相对图像转 90/180/270°，此时「图像左上角那个」并不是符号的左上角。
 *
 * 后果极具迷惑性：定位图形照样全部找到，四角构成的正方形也完全正确，
 * 单应性尺度、每格像素数都对，只是行列被转置了。表现出来就是「能定位、
 * 但头部校验失败」，而且五个档位全失败，看上去像档位判别或几何模型的问题。
 * 真机实测中这一条独占「头部校验失败」的全部案例：把照片转正后，
 * 同一批照片 N=96 与 N=128 全部整帧解出（含 RS 与 CRC）。
 *
 * 修法是逐个试四种朝向。代价可控：判出后随跟踪锁复用，只有失锁帧才重搜。
 * 更彻底的做法是给格式加朝向标记（如 QR 用三个定位图形 + 一个空位），
 * 但那要动协议，且四选一的搜索成本本就不高。
 *
 * perm[o] 给出「符号的 TL/TR/BL/BR 分别落在图像四角的哪一个」。
 */
static const int ORIENT_PERM[4][4] = {
    { 0, 1, 2, 3 },   /* 0°   */
    { 1, 3, 0, 2 },   /* 90°  */
    { 3, 2, 1, 0 },   /* 180° */
    { 2, 0, 3, 1 }    /* 270° */
};

/*
 * 直接读朝向：规约 3.1 规定右下角定位图形的 3×3 芯为红色（v=4），其余三角为黑，
 * 就是给方向消歧用的。找出红芯在图像四角中的哪一个，朝向即可一次定死，
 * 冷启动从「四朝向 × 五档位」降到「一朝向 × 五档位」。
 *
 * 判据用通道比值而非绝对值，对曝光与白平衡免疫：真机实测红芯 R≈180、G/B≈30–60，
 * 黑芯三通道相差不到 5，比值判据留了很大余量。
 * 认不出（0 个或多于 1 个红芯）时返回 -1，调用方退回逐个试探。
 *
 * 芯采样半径按模块尺寸下界取：档位未知，但 u ≥ 边长/121（N 最大 128），
 * 取其 0.8 倍保证落在 3×3 芯内部。
 */
static int orient_from_red_core(const vdd_image *img, const double corners[4][2]) {
    double side = hypot(corners[1][0] - corners[0][0], corners[1][1] - corners[0][1]);
    int rad = (int)(side / 121.0 * 0.8);
    if (rad < 1) rad = 1;
    if (rad > 12) rad = 12;

    int red = -1, n_red = 0;
    for (int k = 0; k < 4; k++) {
        double acc[3] = { 0, 0, 0 };
        int cx = (int)corners[k][0], cy = (int)corners[k][1], np = 0;
        for (int dy = -rad; dy <= rad; dy++) {
            for (int dx = -rad; dx <= rad; dx++) {
                uint8_t p[3];
                px_rgb(img, cx + dx, cy + dy, p);
                acc[0] += p[0]; acc[1] += p[1]; acc[2] += p[2];
                np++;
            }
        }
        double r = acc[0] / np, g = acc[1] / np, b = acc[2] / np;
        if (r > g * 1.6 && r > b * 1.6 && r > 60) { red = k; n_red++; }
    }
    if (n_red != 1) return -1;
    for (int o = 0; o < 4; o++) if (ORIENT_PERM[o][3] == red) return o;
    return -1;
}

static void orient_corners(const double in[4][2], int o, double out[4][2]) {
    for (int k = 0; k < 4; k++) {
        out[k][0] = in[ORIENT_PERM[o][k]][0];
        out[k][1] = in[ORIENT_PERM[o][k]][1];
    }
}

/* 探测结果：几何 + 判决阈值 + 头部 */
typedef struct {
    vdd_geom geom;
    double thr[3], halfgap[3];
    vdd_palette pal;            /* 定标条实测调色板；valid=0 时退回阈值判决 */
    int colors;                 /* 头部调制位：8 或 4 */
    int payload_len;
    int header_disagree;        /* 三副本表决中出现分歧的位数 */
} probe_result;

/*
 * 用给定档位 n 试解头部。只采样定标条（32 格）与头部（120 格），
 * 不做全符号采样，因此适合逐个候选档位试探。
 *
 * 档位判别之所以必要：符号本身不携带 N，而 N 错时定位图形照样能在图像空间
 * 找到（它们只是四个角），单应性却会整体错尺度，格心全数偏掉。
 * 现场表现就是「定位成功但头部校验失败」，靠人工对齐两端档位极易踩坑。
 */
/*
 * 单格判决（不做逐行预计算的直通版本），抽样与退路阈值判决用。
 * 调色板可用时按在用各色最近邻，否则三通道独立阈值；erase 回填是否低置信，
 * 判据与全量判决完全相同（擦除门限 PAL_ERASE / 0.15）。p 须已减去偏移。
 * 返回值即写进位流的那几位：8 色为格值，4 色为 2 bit 值。
 */
static int decide_plain(const probe_result *pr, double s, double t,
                        const double p[3], int *erase) {
    const int four = pr->colors == 4;
    if (four ? pr->pal.valid4 : pr->pal.valid) {
        double conf;
        int v = classify_cell(&pr->pal, s, t, four ? PALETTE4 : COLORS8,
                              four ? 4 : 8, p, &conf);
        *erase = conf < PAL_ERASE;
        return v;
    }
    if (four) {
        /*
         * 4 色的阈值退路：三通道归一化偏离 u 与各组色的 ±1 符号模式做相关，
         * 取最大者。逐通道独立二值判决在这里不适用——单通道翻转会落到组外
         * 的格值上，而组外格值离三个组内色等距，无从判起。
         * 置信度取最大与次大相关值之差的一半：三通道都恰在半间隔上时为 1，
         * 与 8 色退路的 min|u| 同一量纲，擦除门限沿用 0.15。
         */
        double u[3], best = -1e30, second = -1e30;
        int bi = 0;
        for (int ch = 0; ch < 3; ch++) u[ch] = (p[ch] - pr->thr[ch]) / pr->halfgap[ch];
        for (int d = 0; d < 4; d++) {
            double sc = 0;
            for (int ch = 0; ch < 3; ch++) sc += (PALETTE4[d] & CHAN_BIT[ch]) ? u[ch] : -u[ch];
            if (sc > best) { second = best; best = sc; bi = d; }
            else if (sc > second) second = sc;
        }
        *erase = (best - second) * 0.5 < 0.15;
        return bi;
    }
    int v = 0;
    double conf = 1e9;
    for (int ch = 0; ch < 3; ch++) {
        double d = p[ch] - pr->thr[ch];
        if (d > 0) v |= CHAN_BIT[ch];
        double a = fabs(d) / pr->halfgap[ch];
        if (a < conf) conf = a;
    }
    *erase = conf < 0.15;
    return v;
}

/*
 * 从交织位流里取出第 i 块（数据段 + 奇偶段）。seras/eras 非空时回填擦除位置，
 * 返回擦除数；超过 RS_ERAS_MAX 的标记忽略，按错误处理。
 */
static int extract_block(const uint8_t *stream, const uint8_t *seras, int nb,
                         const int *blens, int max_len, int i,
                         uint8_t *block, int *eras) {
    int n_eras = 0, bp = 0, pos = 0;
    /* 数据字节：第 j 轮的本块槽位 */
    for (int j = 0; j < max_len; j++) {
        for (int k = 0; k < nb; k++) {
            if (j >= blens[k]) continue;
            if (k == i) {
                block[bp] = stream[pos];
                if (seras && seras[pos] && n_eras < RS_ERAS_MAX) eras[n_eras++] = bp;
                bp++;
            }
            pos++;
        }
    }
    /* 奇偶字节 */
    for (int j = 0; j < RS_PARITY; j++) {
        for (int k = 0; k < nb; k++) {
            if (k == i) {
                block[bp] = stream[pos];
                if (seras && seras[pos] && n_eras < RS_ERAS_MAX) eras[n_eras++] = bp;
                bp++;
            }
            pos++;
        }
    }
    return n_eras;
}

/* 分阶段耗时回填。各出口都要填，走到哪算到哪 */
static void set_diag_times(vdd_diag *d, double find, double probe, double beta,
                           double sample, double decide, double rs) {
    d->ms_find = find;
    d->ms_probe = probe;
    d->ms_beta = beta;
    d->ms_sample = sample;
    d->ms_decide = decide;
    d->ms_rs = rs;
    d->ms_geom = g_ms_geom;
    d->ms_calib = g_ms_calib;
}

/*
 * 抽样格：坏帧早退与重复帧判定共用。每 PROBE_STEP 个数据格取一个，
 * 少于 PROBE_MIN 个（数据区太小）就不做这两项判定。
 */
#define PROBE_STEP 16
#define PROBE_MIN 64

/*
 * 重复帧门限：不同格数 × DUP_DIFF_DIV <= 抽样数，即不同格不超过 1/4。
 *
 * 标定（抽样数 474–969）：
 *   同一符号的两次拍摄      0.3%（c128_1 与 c128_2）、2.9%（c128_2 与 hdr96，
 *                          后者是 8 px/格 的截屏、纠正 270 字节）
 *   不同符号                85%–90%（金图像四种失真之间、真机六张之间、
 *                          三个诊断包相邻失败帧之间，共 30 余对）
 * 理论值是 7/8 = 87.5%，1/4 离两边都很远；抽样数 500 时 87.5% 的标准差只有
 * 1.5%。一帧要被误判为重复，它本身的误判率得高到 25% 上下——那样的帧本来也
 * 解不出。
 */
#ifndef DUP_DIFF_DIV
#define DUP_DIFF_DIV 4
#endif

/*
 * 坏帧早退门限：低置信格占抽样数的比例超过 SIGNAL_LOW_MUL / SIGNAL_LOW_DIV
 * （1/5）。低置信判据与擦除完全相同（PAL_ERASE / 0.15）。
 *
 * 标定（68 帧：金图像 8、真机照片 8、三个诊断包 28（另 2 帧定位或头部就失败）、
 *       合成局部光照干扰 24）：
 *   至少过一个 RS 块的帧    最高 8.3%（真机，诊断包 N=112 fail08，1/18）；
 *                          合成反光样本最高 11.1%（暗斑 4K 帧，第二遍分区修正后 22/23）
 *   以 VDD_ERR_SIGNAL 收场   12.6%–31.3%，14 帧
 * 取 20%：比能过块的真机帧高出一倍多、比合成干扰帧高出 1.8 倍，宁可漏判——
 * 漏掉的 3 帧（12.6%、16.2%、18.7%）照旧走完全流程，由打包后的字节级擦除
 * 计数兜底。抽样 500 格时 11% 处的标准差约 1.4%，20% 在 6σ 开外。
 */
#ifndef SIGNAL_LOW_MUL
#define SIGNAL_LOW_MUL 1
#endif
#ifndef SIGNAL_LOW_DIV
#define SIGNAL_LOW_DIV 5
#endif

static int probe_grid(const vdd_image *img, int n, double corners[4][2],
                      int32_t *free_idx, probe_result *out) {
    const double cc = FINDER / 2.0;
    double ref[4][2] = { { cc, cc }, { n - cc, cc }, { cc, n - cc }, { n - cc, n - cc } };
    vdd_geom *g = &out->geom;
    g->n = n;
    geom_reset(g);
    if (homography(ref, corners, g->h) != 0) return VDD_ERR_FINDER;
    /* 每格像素数：对角线距离 / 对角格数，决定面积平均的采样点数 */
    double dx = corners[3][0] - corners[0][0];
    double dy = corners[3][1] - corners[0][1];
    double diag_cells = (n - 2 * cc) * 1.41421356;
    g->cell_px = sqrt(dx * dx + dy * dy) / (diag_cells > 1 ? diag_cells : 1);

    /* 用边框环标定边缘弓曲，补上单应性表达不了的径向畸变（带自校验闸门）。
       跟踪态也每帧重测，不随锁复用——原因见 README「试过但退回的优化」 */
    double t0 = clk_ms();
    refine_edges_guarded(img, g);
    double t1 = clk_ms();
    g_ms_geom += t1 - t0;
    if (calib_thresholds(img, g, out->thr, out->halfgap) != 0) {
        g_ms_calib += clk_ms() - t1;
        return VDD_ERR_CALIB;
    }

    build_palette(img, g, &out->pal);
    g_ms_calib += clk_ms() - t1;

    free_cells(n, free_idx);
    uint8_t head[5];
    memset(head, 0, sizeof head);
    out->header_disagree = 0;
    for (int j = 0; j < 40; j++) {
        int votes = 0;
        for (int k = 0; k < 3; k++) {
            int idx = free_idx[j + 40 * k];
            double gx = (idx % n) + 0.5, gy = (idx / n) + 0.5;
            double p[3];
            sample_cell(img, g, gx, gy, p);
            int dark;
            if (out->pal.valid) {
                dark = classify_dark(&out->pal, gx / n, gy / n, p);
            } else {
                /* 退路：头部三通道一致，取多数以抗单通道误读 */
                int d = 0;
                for (int ch = 0; ch < 3; ch++) if (p[ch] < out->thr[ch]) d++;
                dark = d >= 2;
            }
            if (dark) votes++;
        }
        if (votes == 1 || votes == 2) out->header_disagree++;
        if (votes >= 2) head[j >> 3] |= (uint8_t)(0x80 >> (j & 7));
    }
    if (head[0] != MAGIC || (head[1] >> 4) != VERSION || crc8(head, 4) != head[4])
        return VDD_ERR_HEADER;
    /* 低 4 位：bit1..0 调制方式（0 = 8 色，1 = 4 色，2/3 保留），bit3..2 保留为 0。
       调制位在 crc8 覆盖范围内，读错会先被 CRC 拦下 */
    if ((head[1] & 0x0C) || (head[1] & 0x03) > MOD_4) return VDD_ERR_HEADER;
    out->colors = (head[1] & 0x03) == MOD_4 ? 4 : 8;
    out->payload_len = (head[2] << 8) | head[3];
    if (out->payload_len <= 0 || out->payload_len > vdd_net_capacity_colors(n, out->colors))
        return VDD_ERR_CAPACITY;
    return 0;
}

int vdd_decode(const vdd_image *img, int n_grid, vdd_lock *lock,
               uint8_t *frame_out, size_t frame_cap) {
    return vdd_decode_diag(img, n_grid, lock, frame_out, frame_cap, NULL);
}

int vdd_decode_diag(const vdd_image *img, int n_grid, vdd_lock *lock,
                    uint8_t *frame_out, size_t frame_cap, vdd_diag *diag) {
    if (!img || !img->pixels || !frame_out) return VDD_ERR_ARGS;
    if (n_grid != VDD_GRID_AUTO && !is_valid_grid(n_grid)) return VDD_ERR_ARGS;
    gf_init();
    g_ms_geom = 0;
    g_ms_calib = 0;
    double t_stage = clk_ms();
    double ms_find = 0, ms_probe = 0, ms_beta = 0;
    double ms_sample = 0, ms_decide = 0, ms_rs = 0;

    double corners[4][2];
    int rc_find = find_finders(img, lock, corners);
    if (lock) lock->wide = 0;              /* 一次性提示，用完即清 */
    if (rc_find != 0) {
        if (lock && lock->valid) {
            /* 跟踪窗失锁则回退全图 */
            if (find_finders(img, NULL, corners) != 0) {
                if (lock) lock->valid = 0;
                return VDD_ERR_FINDER;
            }
        } else {
            return VDD_ERR_FINDER;
        }
    }

    ms_find = clk_ms() - t_stage;
    t_stage = clk_ms();

    int32_t *free_idx = (int32_t *)malloc((size_t)N_MAX * N_MAX * sizeof(int32_t));
    if (!free_idx) return VDD_ERR_NOMEM;

    /* 候选朝向：优先跟踪锁里的，其次红芯直读的，最后才是逐个试探。
       后两者都只是把正确答案排到前面，全部四个仍留作退路。 */
    int orients[4], n_orients = 0;
    int hint = -1;
    if (lock && lock->valid && lock->orient >= 0 && lock->orient < 4) hint = lock->orient;
    if (hint < 0) hint = orient_from_red_core(img, (const double (*)[2])corners);
    if (hint >= 0) orients[n_orients++] = hint;
    for (int i = 0; i < 4; i++) {
        if (n_orients && orients[0] == i) continue;
        orients[n_orients++] = i;
    }

    /* 候选档位：指定则只试一个；自动则优先复用锁里的，再按规约顺序试 */
    int cands[N_GRIDS + 1];
    int n_cands = 0;
    if (n_grid != VDD_GRID_AUTO) {
        cands[n_cands++] = n_grid;
    } else {
        if (lock && lock->grid && is_valid_grid(lock->grid))
            cands[n_cands++] = lock->grid;
        for (int i = 0; i < N_GRIDS; i++) {
            if (n_cands && cands[0] == GRID_SIZES[i]) continue;
            cands[n_cands++] = GRID_SIZES[i];
        }
    }

    /* 失败时报「走得最远」的那个候选的错误，而非最后一个：
       CALIB 说明几何就不对，HEADER 说明几何与定标都过了但校验没过 */
    probe_result pr, best_pr;
    int rc = VDD_ERR_CALIB;
    int best_stage = -1;
    int used_orient = 0;
    double oc[4][2];
    memset(&best_pr, 0, sizeof best_pr);
    for (int o = 0; o < n_orients && rc != 0; o++) {
        orient_corners(corners, orients[o], oc);
        for (int i = 0; i < n_cands; i++) {
            int r = probe_grid(img, cands[i], oc, free_idx, &pr);
            if (r == 0) { rc = 0; used_orient = orients[o]; break; }
            int stage = (r == VDD_ERR_CALIB) ? 0 : (r == VDD_ERR_HEADER ? 1 : 2);
            if (stage > best_stage) { best_stage = stage; rc = r; best_pr = pr; }
        }
    }
    ms_probe = clk_ms() - t_stage;
    t_stage = clk_ms();
    if (rc == 0) memcpy(corners, oc, sizeof oc);   /* oc 即成功那次的朝向 */
    if (rc != 0) {
        if (diag) {
            memset(diag, 0, sizeof *diag);
            diag->grid = best_pr.geom.n;
            diag->cell_px = best_pr.geom.cell_px;
            for (int ch = 0; ch < 3; ch++) {
                diag->thr[ch] = best_pr.thr[ch];
                diag->halfgap[ch] = best_pr.halfgap[ch];
            }
            /* 走到哪算到哪：判别阶段就失败时，恰恰最需要知道时间花在哪 */
            diag->ms_find = ms_find;
            diag->ms_probe = ms_probe;
            diag->ms_geom = g_ms_geom;
            diag->ms_calib = g_ms_calib;
        }
        free(free_idx);
        if (lock) lock->valid = 0;
        return rc;
    }

    const int n = pr.geom.n;
    vdd_geom geom = pr.geom;
    int payload_len = pr.payload_len;
    if ((size_t)payload_len > frame_cap) { free(free_idx); return VDD_ERR_CAPACITY; }

    /* 横向形状参数：跟踪态复用，否则用置信度搜索标定 */
    if (lock && lock->valid && lock->beta > 0.0 && lock->grid == n) {
        geom.beta = lock->beta;
    } else {
        refine_beta(img, &geom);
    }
    ms_beta = clk_ms() - t_stage;
    t_stage = clk_ms();

    double *thr = pr.thr, *halfgap = pr.halfgap;

    /*
     * 色数相关的一切从这里开始分叉：每格位数、判决用的颜色集合、调色板可用性，
     * 以及偏移校正与分区矩匹配里「在用各色等概率」的那个集合。头部判决与档位
     * 探测都在 probe_grid 里、调制位读出之前，两档共用。
     */
    const int colors = pr.colors;
    const int bpc = colors == 4 ? 2 : 3;
    const int *vs = colors == 4 ? PALETTE4 : COLORS8;
    const int nv = colors;
    const int pal_ok = colors == 4 ? pr.pal.valid4 : pr.pal.valid;

    if (diag) {
        memset(diag, 0, sizeof *diag);
        diag->grid = n;
        diag->colors = colors;
        diag->cell_px = geom.cell_px;
        diag->beta = geom.beta;
        double bow = 0;
        for (int i = 0; i < EDGE_SEGS; i++) {
            bow += fabs(geom.prof[0][i] - geom.prof[1][i]);
            bow += fabs(geom.prof[2][i] - geom.prof[3][i]);
        }
        diag->bow = bow / (2 * EDGE_SEGS);
        diag->header_disagree = pr.header_disagree;
        diag->payload_len = payload_len;
        for (int ch = 0; ch < 3; ch++) {
            diag->thr[ch] = thr[ch];
            diag->halfgap[ch] = halfgap[ch];
        }
        diag->probe_diff = -1;
    }

    /*
     * 承载位流的只有 free_idx[HEADER_CELLS, last)：头部已在 probe_grid 里判完，
     * 角部块、边框环、定标条与尾部未用格的判决值从来没人读。采样与判决
     * 只跑这一段，N=128 省掉约一成的格。
     */
    int n_free = free_cells(n, free_idx);
    int nb = (payload_len + RS_DATA_MAX - 1) / RS_DATA_MAX;
    int stream_len = payload_len + RS_PARITY * nb;
    int last = HEADER_CELLS + (int)(((long)stream_len * 8 + bpc - 1) / bpc);
    if (last > n_free) last = n_free;
    const int n_data = last - HEADER_CELLS;

    /* 工作区：采样值 + 格值 + 擦除标记。判决要用到全符号的统计量，
       采样与判决拆成两遍。double 放在最前保证对齐。 */
    uint8_t *ws = (uint8_t *)malloc((size_t)n * n * 3 * sizeof(double)
                                    + (size_t)n * n * 4);
    if (!ws) { free(free_idx); return VDD_ERR_NOMEM; }
    double *cellp = (double *)(void *)ws;           /* 每格三通道采样值 */
    uint8_t *cellv = ws + (size_t)n * n * 3 * sizeof(double);   /* 最终判决 */
    uint8_t *celle = cellv + (size_t)n * n;         /* 1 = 低置信度(擦除) */
    uint8_t *cellv1 = celle + (size_t)n * n;        /* 第一遍（整体偏移）判决 */
    uint8_t *celle1 = cellv1 + (size_t)n * n;

    /*
     * 抽样：数据格序列里每 pstep 个取一个。数据格按行优先排列，等步长抽样
     * 自然覆盖上下全程——LCD 逐行刷新，换帧过渡带是按行分布的，只抽一部分行
     * 会看漏。每行格数与步长不整除，相邻行的抽样列互相错开，左右也覆盖得到。
     * 抽样位置只取决于档位与载荷长度，重复帧判定靠这一点跨帧比对。
     */
    int pstep = PROBE_STEP;
    while (n_data / pstep > VDD_PROBE_MAX) pstep++;
    int n_probe = 0, probe_low = 0, probe_diff = -1;
    uint8_t probe_v[VDD_PROBE_MAX];
    if (n_data / pstep >= PROBE_MIN) {
        double pbias[3] = { 0, 0, 0 }, sm[3] = { 0, 0, 0 }, sr[3] = { 0, 0, 0 };
        for (int i = HEADER_CELLS + pstep / 2; i < last && n_probe < VDD_PROBE_MAX; i += pstep) {
            int idx = free_idx[i];
            double *p = &cellp[(size_t)idx * 3];
            sample_cell(img, &geom, (idx % n) + 0.5, (idx / n) + 0.5, p);
            for (int k = 0; k < 3; k++) sm[k] += p[k];
            if (pal_ok) {
                double s = ((idx % n) + 0.5) / n, t = ((idx / n) + 0.5) / n;
                for (int v = 0; v < nv; v++) {
                    double ref[3];
                    palette_at(&pr.pal, s, t, vs[v], ref);
                    for (int k = 0; k < 3; k++) sr[k] += ref[k] / nv;
                }
            }
            n_probe++;
        }
        /* 与全量判决同一套偏移校正，抽样上的判决才和全量第一遍可比 */
        if (pal_ok)
            for (int k = 0; k < 3; k++) pbias[k] = (sm[k] - sr[k]) / n_probe;
        for (int j = 0; j < n_probe; j++) {
            int idx = free_idx[HEADER_CELLS + pstep / 2 + j * pstep];
            double p[3];
            for (int k = 0; k < 3; k++) p[k] = cellp[(size_t)idx * 3 + k] - pbias[k];
            double s = ((idx % n) + 0.5) / n, t = ((idx / n) + 0.5) / n;
            int er;
            probe_v[j] = (uint8_t)decide_plain(&pr, s, t, p, &er);
            probe_low += er;
        }
        if (lock && lock->valid && lock->dup_n == n_probe &&
            lock->dup_grid == n && lock->dup_len == payload_len &&
            lock->dup_colors == colors) {
            probe_diff = 0;
            for (int j = 0; j < n_probe; j++) probe_diff += probe_v[j] != lock->dup_v[j];
        }
    }
    if (diag) {
        diag->probe_cells = n_probe;
        diag->probe_low = probe_low;
        diag->probe_diff = probe_diff;
    }

    /*
     * 重复帧：抽样格判决值与上一成功帧几乎全同。不同符号的数据格是 RS 编码后
     * 的随机字节，八色等概率，两帧同位置同色的概率只有 1/8，不同格占约 87.5%
     * （4 色档是 1/4，不同格约 75%，离门限 1/4 同样很远）；
     * 同一符号两次扫描只差两帧各自的误判，能解出的帧误判率在几个百分点。
     * 门限取 1/4，两边各留一大截余量（见 DUP_DIFF_DIV 处的标定记录）。
     * 判重放在坏帧早退之前：静止画面下质量一般的重复帧应当算「已收到」，
     * 而不是算失败去触发曝光解锁与重新对焦。
     */
    if (probe_diff >= 0 && probe_diff * DUP_DIFF_DIV <= n_probe) {
        ms_sample = clk_ms() - t_stage;
        if (diag) set_diag_times(diag, ms_find, ms_probe, ms_beta, ms_sample, 0, 0);
        /* 角点照常跟踪；档位、朝向、β 与重复帧记录都不变 */
        for (int k = 0; k < 4; k++) {
            lock->corners[k][0] = corners[k][0];
            lock->corners[k][1] = corners[k][1];
        }
        free(ws); free(free_idx);
        return VDD_DUPLICATE;
    }

    /*
     * 坏帧早退：低置信格比例明显超标，直接放弃，不做全采样、判决与纠错。
     * 这类帧多是屏幕换帧过渡期抓到的（真机 N=128 诊断包里占 44%），
     * 原先要把整帧跑完才在打包后被字节级擦除计数拦下。门限见 SIGNAL_LOW_DIV。
     * 抽样判决只做了整体偏移、没做分区修正，门限的余量已把这一点算进去。
     */
    if (n_probe > 0 && probe_low * SIGNAL_LOW_DIV > n_probe * SIGNAL_LOW_MUL) {
        ms_sample = clk_ms() - t_stage;
        if (diag) {
            diag->rs_blocks = nb;
            set_diag_times(diag, ms_find, ms_probe, ms_beta, ms_sample, 0, 0);
        }
        free(ws); free(free_idx);
        return VDD_ERR_SIGNAL;
    }

    /* 全量采样（抽样格已采过，跳过） */
    for (int i = HEADER_CELLS; i < last; i++) {
        int rel = i - HEADER_CELLS;
        if (n_probe && rel % pstep == pstep / 2 && rel / pstep < n_probe) continue;
        int idx = free_idx[i];
        sample_cell(img, &geom, (idx % n) + 0.5, (idx / n) + 0.5, &cellp[(size_t)idx * 3]);
    }

    ms_sample = clk_ms() - t_stage;
    t_stage = clk_ms();

    /*
     * 判决偏移校正。载荷是 RS 编码后的字节流，八色近似等概率，因此所有
     * 数据格的三通道均值应当等于同位置调色板八色的均值。两者之差就是这
     * 一帧数据区相对定标条的整体偏移。
     *
     * 为什么需要它：四条定标条在每一帧里内容相同，屏幕换帧过渡期的新旧
     * 帧混合、杂散光、相邻格串扰都只改变数据区，定标条读数照旧。于是
     * 调色板一切指标正常（八色间距 90 以上），判决却整体偏向亮端——
     * 实测坏帧里黑色格只剩 3.6%、白色格高达 25.9%，本该各 12.5%。
     * 把这个差值减掉，八色分布随即回到 12% 附近。
     *
     * 它现在是第一遍判决的偏移，也是分区修正在样本不足时收缩回去的目标。
     * 八色均值也是双线性的，先把四角八色平均好，每格只算一次插值。
     */
    /* 4 色档同理，只是等概率的是在用的四色，均值只能按这四色算，否则整体偏 */
    double bias[3] = { 0, 0, 0 };
    if (pal_ok && n_data > 0) {
        vdd_palette pm;
        for (int q = 0; q < 4; q++)
            for (int k = 0; k < 3; k++) {
                double a = 0;
                for (int v = 0; v < nv; v++) a += pr.pal.c[q][vs[v]][k];
                pm.c[q][0][k] = a / nv;
            }
        double sm[3] = { 0, 0, 0 }, sr[3] = { 0, 0, 0 };
        for (int i = HEADER_CELLS; i < last; i++) {
            int idx = free_idx[i];
            double ref[3];
            palette_at(&pm, ((idx % n) + 0.5) / n, ((idx / n) + 0.5) / n, 0, ref);
            for (int k = 0; k < 3; k++) {
                sr[k] += ref[k];
                sm[k] += cellp[(size_t)idx * 3 + k];
            }
        }
        for (int k = 0; k < 3; k++) bias[k] = (sm[k] - sr[k]) / n_data;
    }

    /* 分区数按数据格数自适应：每区每色平均至少约 50 个样本 */
    int zn = 0;
    if (pal_ok) {
        zn = (int)sqrt(n_data / 400.0);
        if (zn > ZONE_MAX) zn = ZONE_MAX;
        if (zn < 2) zn = 0;
    }
    /* 每区每通道的矩：格数、Σobs、Σobs²、Σ调色板八色均值、Σ调色板八色均方 */
    double zs[ZONE_MAX][ZONE_MAX][3][4];
    int zc_n[ZONE_MAX][ZONE_MAX];
    if (zn) { memset(zs, 0, sizeof zs); memset(zc_n, 0, sizeof zc_n); }

    /*
     * 第一遍判决 + 置信度。调色板可用时按实测八色（逐行预计算）+ 整体偏移
     * 做最近邻；否则退回三通道独立阈值。顺带按区累加矩。
     */
    {
        pal_row prow;
        int cur_r = -1;
        for (int i = HEADER_CELLS; i < last; i++) {
            int idx = free_idx[i];
            int r = idx / n, c = idx % n;
            const double *p = &cellp[(size_t)idx * 3];
            double s = (c + 0.5) / n;
            if (!pal_ok) {
                double q[3] = { p[0], p[1], p[2] };
                int er;
                cellv1[idx] = (uint8_t)decide_plain(&pr, s, (r + 0.5) / n, q, &er);
                celle1[idx] = (uint8_t)er;
                continue;
            }
            if (r != cur_r) { pal_row_init(&pr.pal, (r + 0.5) / n, vs, nv, &prow); cur_r = r; }
            double ref[8][3];
            for (int v = 0; v < nv; v++)
                for (int k = 0; k < 3; k++)
                    ref[v][k] = prow.a[v][k] + s * prow.b[v][k] + bias[k];
            double conf;
            int v = classify_ref((const double (*)[3])ref, nv, p, &conf);
            cellv1[idx] = (uint8_t)v;
            celle1[idx] = conf < PAL_ERASE ? 1 : 0;
            if (zn) {
                int zr = r * zn / n, zcol = c * zn / n;
                zc_n[zr][zcol]++;
                for (int k = 0; k < 3; k++) {
                    double *z4 = zs[zr][zcol][k];
                    z4[0] += p[k]; z4[1] += p[k] * p[k];
                    z4[2] += prow.ma[k] + s * prow.mb[k];
                    z4[3] += prow.qaa[k] + s * (2 * prow.qab[k] + s * prow.qbb[k]);
                }
            }
        }
    }

    /* 第二遍：分区仿射修正后的调色板重判 */
    if (zn) {
        zone_delta zd;
        zd.z = zn;
        for (int zr = 0; zr < zn; zr++)
            for (int zc = 0; zc < zn; zc++)
                for (int k = 0; k < 3; k++) {
                    double ga = 1.0, gb = bias[k];
                    int cnt = zc_n[zr][zc];
                    if (cnt >= ZONE_MIN_CELLS) {
                        const double *z4 = zs[zr][zc][k];
                        double mo = z4[0] / cnt, vo = z4[1] / cnt - mo * mo;
                        double mp = z4[2] / cnt, vp = z4[3] / cnt - mp * mp;
                        if (vp > 1.0 && vo > 0) {
                            ga = sqrt(vo / vp);
                            if (ga < 0.25) ga = 0.25;
                            if (ga > 2.5) ga = 2.5;
                            gb = mo - ga * mp;
                        }
                    }
                    zd.d[zr][zc][0][k] = ga;
                    zd.d[zr][zc][1][k] = gb;
                }
        pal_row prow;
        double zrow[ZONE_MAX][2][3];
        int cur_r = -1;
        for (int i = HEADER_CELLS; i < last; i++) {
            int idx = free_idx[i];
            int r = idx / n, c = idx % n;
            double s = (c + 0.5) / n;
            if (r != cur_r) {
                double t = (r + 0.5) / n;
                pal_row_init(&pr.pal, t, vs, nv, &prow);
                zone_row(&zd, t, zrow);
                cur_r = r;
            }
            int c0, c1;
            double w;
            zone_lerp(zn, s, &c0, &c1, &w);
            double ga[3], gb[3];
            for (int k = 0; k < 3; k++) {
                ga[k] = zrow[c0][0][k] * (1 - w) + zrow[c1][0][k] * w;
                gb[k] = zrow[c0][1][k] * (1 - w) + zrow[c1][1][k] * w;
            }
            double ref[8][3];
            for (int v = 0; v < nv; v++)
                for (int k = 0; k < 3; k++)
                    ref[v][k] = ga[k] * (prow.a[v][k] + s * prow.b[v][k]) + gb[k];
            double conf;
            cellv[idx] = (uint8_t)classify_ref((const double (*)[3])ref, nv,
                                               &cellp[(size_t)idx * 3], &conf);
            celle[idx] = conf < PAL_ERASE ? 1 : 0;
        }
    }
    else {
        cellv = cellv1;
        celle = celle1;
    }
    if (diag) diag->zones = zn;
    ms_decide = clk_ms() - t_stage;
    t_stage = clk_ms();

    /*
     * 数据格 -> 位流 -> 交织字节 + 字节级擦除标记。
     *
     * 分区修正启用时两遍判决各出一条位流：第二遍优先，某块纠不动再拿第一遍
     * （即改造前的整体偏移判决）的同一块重试。分区修正在局部光照干扰下收益
     * 很大，但在没有这类干扰的帧上只是把误判挪个位置，个别块会从「刚好纠得动」
     * 变成「刚好纠不动」；逐块回退保证通过块数不少于改造前。多试一条位流的
     * 伪成功风险由 RS_ERAS_MAX 留的 8 个复验符号兜住（约 2^-64 量级翻倍）。
     */
    const int n_streams = zn ? 2 : 1;
    uint8_t stream[2][8192];
    uint8_t seras[2][8192];
    if (stream_len > (int)sizeof stream[0]) { free(ws); free(free_idx); return VDD_ERR_CAPACITY; }
    long total_bits = (long)stream_len * 8;
    int n_bad = stream_len;
    for (int si = 0; si < n_streams; si++) {
        const uint8_t *vv = si ? cellv1 : cellv, *ee = si ? celle1 : celle;
        memset(stream[si], 0, (size_t)stream_len);
        memset(seras[si], 0, (size_t)stream_len);
        long bitpos = 0;
        for (int i = HEADER_CELLS; i < last && bitpos < total_bits; i++) {
            int idx = free_idx[i];
            uint8_t v = vv[idx];
            uint8_t er = ee[idx];
            for (int b = bpc - 1; b >= 0 && bitpos < total_bits; b--) {
                if ((v >> b) & 1) stream[si][bitpos >> 3] |= (uint8_t)(0x80 >> (bitpos & 7));
                if (er) seras[si][bitpos >> 3] = 1;
                bitpos++;
            }
        }
        int nbad = 0;
        for (int i = 0; i < stream_len; i++) nbad += seras[si][i];
        if (nbad < n_bad) n_bad = nbad;
    }

    /*
     * 无望帧提前退出。屏幕换帧过渡期抓到的那一帧，新旧两帧混合会让格间
     * 对比整体塌陷，判决大面积失效——实测这类帧有 41%~48% 的字节含低置信
     * 格，而能解出的帧只有 8%~11%，两者之间没有交叠。25% 已经远超 RS 的
     * 物理上限（每块 255 字节只有 32 个校验符号），继续跑 13 个块的两轮
     * 纠错纯属浪费，省下的时间直接变成更高的送检帧率。
     * 大部分这类帧现在已在全采样前被抽样拦下，这里是兜底。两条位流取较好的。
     */
    if (n_bad > stream_len / 4) {
        /* 与下面的纠错失败出口一致：不动跟踪锁，几何是对的。
           rs_erasures 这里报的是未截断的候选数，读数远超 nb*RS_ERAS_MAX
           即可认定是这条路径。 */
        ms_rs = clk_ms() - t_stage;
        if (diag) {
            diag->rs_blocks = nb;
            diag->rs_erasures = n_bad;
            set_diag_times(diag, ms_find, ms_probe, ms_beta, ms_sample, ms_decide, ms_rs);
        }
        free(ws); free(free_idx);
        return VDD_ERR_SIGNAL;
    }

    /* 解交织 + 分块 errata 纠错 */
    int base = payload_len / nb;
    int rem = payload_len % nb;
    int blens[64];
    if (nb > 64) { free(ws); free(free_idx); return VDD_ERR_CAPACITY; }
    int max_len = 0;
    for (int i = 0; i < nb; i++) {
        blens[i] = base + (i >= nb - rem ? 1 : 0);
        if (blens[i] > max_len) max_len = blens[i];
    }
    uint8_t block[255];
    int eras[RS_ERAS_MAX];
    /*
     * RS 统计：即使中途有块纠错失败也要把剩余块跑完再返回错误。
     * 一遇失败就 return 的话，rs_blocks_ok 永远等于失败块的下标，
     * 「22 块过 21 块」与「22 块过 0 块」在读数上无法区分。
     */
    int rs_failed = 0;
    if (diag) {
        diag->rs_blocks = nb;
        diag->rs_blocks_ok = 0;
        diag->rs_corrected = 0;
        diag->rs_erasures = 0;
    }
    int out_off = 0;
    for (int i = 0; i < nb; i++) {
        int bl = blens[i];
        int n_fixed = 0;
        int block_ok = 0;
        for (int si = 0; si < n_streams && !block_ok; si++) {
            int n_eras = extract_block(stream[si], seras[si], nb, blens, max_len, i, block, eras);
            /* 擦除标记数按「喂进 RS 的那一次」统计（首选位流），与置信度门限直接对应 */
            if (si == 0 && diag) diag->rs_erasures += n_eras;
            if (rs_errata_decode(block, bl + RS_PARITY, eras, n_eras, &n_fixed) == 0) {
                block_ok = 1;
                break;
            }
            /* 擦除信息可能误导；退一步用纯错误模式再试一次。
               block 已被就地修改，重新抽取 */
            extract_block(stream[si], NULL, nb, blens, max_len, i, block, NULL);
            if (rs_errata_decode(block, bl + RS_PARITY, NULL, 0, &n_fixed) == 0) block_ok = 1;
        }
        if (!block_ok) rs_failed = 1;
        if (block_ok && diag) {
            diag->rs_blocks_ok++;
            diag->rs_corrected += n_fixed;
        }
        /* 失败块照样落到输出缓冲：反正整帧要以 VDD_ERR_RS 收场，
           这里不分叉是为了让后续块的偏移保持正确 */
        memcpy(frame_out + out_off, block, (size_t)bl);
        out_off += bl;
    }
    if (rs_failed) {
        /* 不动跟踪锁：几何是对的（头部已过），只是这一帧信噪比不够，
           丢锁反而会让下一帧退回全图搜索 —— 沿用改造前的行为 */
        ms_rs = clk_ms() - t_stage;
        if (diag) set_diag_times(diag, ms_find, ms_probe, ms_beta, ms_sample, ms_decide, ms_rs);
        free(ws); free(free_idx);
        return VDD_ERR_RS;
    }

    ms_rs = clk_ms() - t_stage;
    if (diag) set_diag_times(diag, ms_find, ms_probe, ms_beta, ms_sample, ms_decide, ms_rs);

    if (lock) {
        for (int k = 0; k < 4; k++) {
            lock->corners[k][0] = corners[k][0];
            lock->corners[k][1] = corners[k][1];
        }
        lock->beta = geom.beta;
        lock->grid = n;              /* 判别出的档位随锁复用，后续帧不再逐个试 */
        lock->orient = used_orient;  /* 朝向同理，避免每帧四选一 */
        lock->valid = 1;
        /* 重复帧记录：抽样位置上的最终判决值 */
        lock->dup_grid = n;
        lock->dup_len = payload_len;
        lock->dup_colors = colors;
        lock->dup_n = n_probe;
        for (int j = 0; j < n_probe; j++)
            lock->dup_v[j] = cellv[free_idx[HEADER_CELLS + pstep / 2 + j * pstep]];
    }
    free(ws); free(free_idx);
    return payload_len;
}
