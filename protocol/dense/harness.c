/*
 * VD-D1 解码 harness：拿一批 24 位 BMP 直接喂 vd_dense_core，打印每帧的解码
 * 结果、分阶段耗时与 RS 观测量。优化解码器时用它同时看「快了多少」和
 * 「准确性有没有退步」——这两件事必须一起看，本仓库已经有两次「明显更快」
 * 的改动实测把原本整帧解出的真机照片打成 0 块通过（见 DenseL1/README 的
 * 「试过但退回的优化」）。
 *
 * 编译：
 *   clang -O2 -std=c99 -o vddh protocol/dense/harness.c \
 *         receiver/QrBinary/DenseL1/vd_dense_core.c -lm
 *
 * 用法：
 *   ./vddh [-r 次数] [-l] [-x] <图.bmp> [更多.bmp ...]
 *     -r  每帧重复解码取最快值，默认 5；测耗时时用，测正确性时 1 就够
 *     -l  跨帧复用跟踪锁，模拟真机连续取景；默认每帧独立全图搜索。
 *         重复解码时每次都从同一份锁出发，否则第二次起都会被判成重复帧
 *     -x  （配合 -l）每帧前清掉锁里的重复帧记录，强制走全流程。
 *         用来交叉核对：同一串帧带/不带 -x 各跑一遍，被判重复的帧在 -x 下
 *         解出的字节摘要应与上一个成功帧相同
 *
 * 输出里的「抽样」是全采样前的那批抽样格：低置信格数 / 抽样数，以及与上一
 * 成功帧的不同格数（-1 = 无可比记录）。坏帧早退与重复帧判定的门限就看它。
 *
 * 回归样本：
 *   protocol/dense/golden/ 下的 bmp   合成金图像，任何改动都不许退步
 *   testdata/                       真机实拍（不入库，见 .gitignore）
 *   诊断包里的 fail*.bmp            真机失败帧，改动的主要检验对象
 *
 * BMP 而非 PNG：格式极简、无压缩，App 的「导出诊断帧 / 诊断包」直接产出这个
 * 格式，导出的帧可以原样喂进来复现，不必重建现场。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../receiver/QrBinary/DenseL1/vd_dense_core.h"

/* 读 24 位 BMP 到紧凑 RGB24。只认最常见的那一种：无压缩、bpp=24。 */
static uint8_t *load_bmp(const char *path, int *out_w, int *out_h) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    uint8_t hd[54];
    if (fread(hd, 1, sizeof hd, f) != sizeof hd) { fclose(f); return NULL; }
    if (hd[0] != 'B' || hd[1] != 'M') { fclose(f); return NULL; }
    int off = hd[10] | hd[11] << 8 | hd[12] << 16 | hd[13] << 24;
    int w = hd[18] | hd[19] << 8 | hd[20] << 16 | hd[21] << 24;
    int h = hd[22] | hd[23] << 8 | hd[24] << 16 | hd[25] << 24;
    int bpp = hd[28] | hd[29] << 8;
    if (bpp != 24 || w <= 0) { fclose(f); return NULL; }
    /* 高度为正表示自底向上存储 */
    int bottom_up = h > 0;
    if (h < 0) h = -h;

    size_t row = ((size_t)w * 3 + 3) & ~(size_t)3;
    uint8_t *raw = (uint8_t *)malloc(row * (size_t)h);
    if (!raw) { fclose(f); return NULL; }
    if (fseek(f, off, SEEK_SET) != 0 || fread(raw, 1, row * (size_t)h, f) != row * (size_t)h) {
        free(raw); fclose(f); return NULL;
    }
    fclose(f);

    uint8_t *out = (uint8_t *)malloc((size_t)w * h * 3);
    if (!out) { free(raw); return NULL; }
    for (int y = 0; y < h; y++) {
        const uint8_t *s = raw + (size_t)(bottom_up ? (h - 1 - y) : y) * row;
        uint8_t *d = out + (size_t)y * w * 3;
        for (int x = 0; x < w; x++) {           /* BMP 是 BGR */
            d[x * 3 + 0] = s[x * 3 + 2];
            d[x * 3 + 1] = s[x * 3 + 1];
            d[x * 3 + 2] = s[x * 3 + 0];
        }
    }
    free(raw);
    *out_w = w; *out_h = h;
    return out;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static const char *err_name(int rc) {
    switch (rc) {
    case VDD_ERR_ARGS:     return "参数非法";
    case VDD_ERR_FINDER:   return "定位失败";
    case VDD_ERR_HEADER:   return "头部校验失败";
    case VDD_ERR_RS:       return "纠错失败";
    case VDD_ERR_CAPACITY: return "容量越界";
    case VDD_ERR_NOMEM:    return "分配失败";
    case VDD_ERR_CALIB:    return "定标失败";
    case VDD_ERR_SIGNAL:   return "信噪比过低";
    case VDD_DUPLICATE:    return "重复帧";
    default:               return "未知";
    }
}

int main(int argc, char **argv) {
    int repeat = 5, use_lock = 0, no_dup = 0, first = 1;
    for (; first < argc; first++) {
        if (!strcmp(argv[first], "-l")) { use_lock = 1; continue; }
        if (!strcmp(argv[first], "-x")) { no_dup = 1; continue; }
        if (!strcmp(argv[first], "-r") && first + 1 < argc) {
            repeat = atoi(argv[++first]);
            if (repeat < 1) repeat = 1;
            continue;
        }
        break;
    }
    if (first >= argc) {
        fprintf(stderr, "用法: %s [-r 次数] [-l] [-x] <图.bmp> [...]\n", argv[0]);
        return 2;
    }

    vdd_lock lock;
    memset(&lock, 0, sizeof lock);
    int total = 0, decoded = 0, dups = 0;

    for (int a = first; a < argc; a++) {
        int w, h;
        uint8_t *px = load_bmp(argv[a], &w, &h);
        const char *base = strrchr(argv[a], '/');
        base = base ? base + 1 : argv[a];
        if (!px) { printf("%-28s 读取失败\n", base); continue; }

        vdd_image img = { px, w, h, (size_t)w * 3, 0 };
        uint8_t frame[8192];
        vdd_diag d, best_d;
        int rc = 0;
        double best = 1e30;
        if (no_dup) lock.dup_n = 0;
        /* 每次重复都从同一份锁出发：跟踪锁会被成功帧回写，不复原的话
           第二次起测的就是「重复帧」路径，不是这一帧本身 */
        vdd_lock saved = lock;
        for (int i = 0; i < repeat; i++) {
            lock = saved;
            memset(&d, 0, sizeof d);
            double t0 = now_ms();
            rc = vdd_decode_diag(&img, VDD_GRID_AUTO, use_lock ? &lock : NULL,
                                 frame, sizeof frame, &d);
            /* 与 DenseDecoder.run 一致：跟踪态纠错失败就放大搜索窗重来一次 */
            if (use_lock && rc == VDD_ERR_RS && lock.valid) {
                lock.wide = 1;
                memset(&d, 0, sizeof d);
                rc = vdd_decode_diag(&img, VDD_GRID_AUTO, &lock, frame, sizeof frame, &d);
            }
            double dt = now_ms() - t0;
            /* 分段耗时取最快那一次的，与总耗时同源，否则两者对不上 */
            if (dt < best) { best = dt; best_d = d; }
        }
        d = best_d;
        total++;
        if (rc > 0) decoded++;
        if (rc == VDD_DUPLICATE) dups++;
        /* 帧字节摘要（FNV-1a），核对重复帧判定与「同一符号两次扫描」用 */
        uint32_t fnv = 2166136261u;
        if (rc > 0)
            for (int i = 0; i < rc; i++) { fnv ^= frame[i]; fnv *= 16777619u; }

        printf("%-28s %dx%d  ", base, w, h);
        if (rc > 0) printf("解出 %d 字节 #%08x", rc, fnv);
        else        printf("%s(%d)", err_name(rc), rc);
        /* 色数来自头部调制位：8 = 8 色档，4 = 4 色降级档，0 = 头部未过 */
        printf("  N=%d %d色 cell=%.2f  RS %d/%d 纠正%d 擦除%d  抽样 低%d/%d 异%d 分区%d\n",
               d.grid, d.colors, d.cell_px, d.rs_blocks_ok, d.rs_blocks,
               d.rs_corrected, d.rs_erasures,
               d.probe_low, d.probe_cells, d.probe_diff, d.zones);
        /* 前六段互不重叠，加起来约等于整帧；后两段是 probe 内部细分 */
        printf("    耗时 %.2f ms = 定位%.2f 判别%.2f β%.2f 采样%.2f 判决%.2f 纠错%.2f"
               "  [probe 内: 几何%.2f 定标%.2f]\n",
               best, d.ms_find, d.ms_probe, d.ms_beta,
               d.ms_sample, d.ms_decide, d.ms_rs, d.ms_geom, d.ms_calib);
        free(px);
    }

    printf("\n合计 %d 张，整帧解出 %d 张，重复帧 %d 张\n", total, decoded, dups);
    return 0;
}
