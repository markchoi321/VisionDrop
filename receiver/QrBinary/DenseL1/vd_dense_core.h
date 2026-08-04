/*
 * VD-D1 密集彩色 L1 解码核心（纯 C99，零依赖）
 *
 * 规约见 protocol/dense/DENSE.md，行为参照 protocol/dense/dense_refimpl.py。
 * 输入一帧图像，输出一个 L2 帧的字节；之后交给现有 Swift 协议栈
 * （FrameParser -> 解码器），L2-L5 不感知物理层差异。
 *
 * 设计取向：
 * - 纯 C、无堆外依赖、单次 malloc 工作区，编译器可自动 NEON 向量化；
 * - 亮度域游程扫描找定位图形（无 ML 推理），锁定后可用 hint 跳过全图搜索；
 * - RS(255,223+32) 带擦除的 errata 解码：低置信度格标记为擦除，
 *   纠错能力相对纯错误模式接近翻倍。
 */
#ifndef VD_DENSE_CORE_H
#define VD_DENSE_CORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回码：非负 = 帧字节数，负值 = 错误 */
enum {
    VDD_ERR_ARGS = -1,          /* 参数非法 */
    VDD_ERR_FINDER = -2,        /* 定位图形不足 4 个 */
    VDD_ERR_HEADER = -3,        /* 头部魔数/版本/CRC 校验失败 */
    VDD_ERR_RS = -4,            /* RS 纠错失败 */
    VDD_ERR_CAPACITY = -5,      /* 声明的载荷长度超出该 N 的容量 */
    VDD_ERR_NOMEM = -6,         /* 工作区分配失败 */
    VDD_ERR_CALIB = -7,         /* 定标条对比度不足，无法判决（此前混在 HEADER 里） */
    /*
     * 信噪比过低，整帧放弃（低置信格超过四分之一）。与 VDD_ERR_RS 分开是为了
     * 让调用方省掉无谓的重试：这类帧多半是屏幕换帧过渡期抓到的，数据区被新旧
     * 两帧混合污染，换个定位起点重来一次结果一样，而 4K 下一次全图搜索不便宜。
     */
    VDD_ERR_SIGNAL = -8
};

/* n_grid 传此值表示自动判别档位（逐个候选试到头部校验通过） */
#define VDD_GRID_AUTO 0

/* 输入图像：交错像素。camera 路径用 BGRA（AVFoundation 硬件转换输出），
   测试路径用 RGB24。 */
typedef struct {
    const uint8_t *pixels;
    int width;
    int height;
    size_t stride;              /* 每行字节数 */
    int is_bgra;                /* 1 = BGRA32，0 = RGB24 */
} vdd_image;

/* 定位结果，可跨帧复用作 hint（跟踪态跳过全图搜索） */
typedef struct {
    double corners[4][2];       /* 左上/右上/左下/右下 中心像素坐标 (x,y) */
    double beta;                /* 已标定的横向形状参数；0 = 未标定 */
    int grid;                   /* 已判别出的档位；0 = 未判别 */
    int orient;                 /* 已判别出的符号朝向 0/1/2/3（×90°）*/
    int valid;
} vdd_lock;

/* 上一次解码的测量值，供界面诊断与真机标定 */
typedef struct {
    int grid;                   /* 实际使用的档位 */
    double cell_px;             /* 每格像素数 */
    double beta;                /* 横向形状参数 */
    double thr[3];              /* 三通道判决阈值 */
    double halfgap[3];          /* 三通道半间隔，越小越接近失效 */
    double bow;                 /* 弓曲幅度（格），反映镜头畸变强度 */
    int header_disagree;        /* 头部 40 位中三副本不一致的位数 */
    int payload_len;
    /*
     * RS 层观测量。只返回一个 VDD_ERR_RS 的话，无法区分「差一点」与「差很远」：
     * 22 块过 21 块和 22 块过 0 块是完全不同的两种故障。失败时这几个数照样回填，
     * 且失败后仍把剩余块跑完，读数才有可比性。
     */
    int rs_blocks;              /* RS 块总数 */
    int rs_blocks_ok;           /* 纠错通过的块数 */
    int rs_corrected;           /* 通过的块里纠正的字节数合计 */
    int rs_erasures;            /* 实际喂给 RS 的擦除标记总数（全部块合计） */
} vdd_diag;

/*
 * 解码一个 N 格符号。
 *   img       输入图像
 *   n_grid    符号边长（格），须为规约档位之一；传 VDD_GRID_AUTO 自动判别
 *   lock      可为 NULL。valid=1 时只在四角附近小窗精定位（跟踪态），
 *             并复用已判别的档位与形状参数；解码成功后回写。
 *   frame_out 输出缓冲
 *   frame_cap 输出缓冲容量
 * 返回帧字节数（>0）或负错误码。
 */
int vdd_decode(const vdd_image *img, int n_grid, vdd_lock *lock,
               uint8_t *frame_out, size_t frame_cap);

/* 同上，额外回填最近一次的测量值（diag 可为 NULL） */
int vdd_decode_diag(const vdd_image *img, int n_grid, vdd_lock *lock,
                    uint8_t *frame_out, size_t frame_cap, vdd_diag *diag);

/* 该 N 的净载荷上限（字节），非法 N 返回 0。与规约 6 节一致。 */
int vdd_net_capacity(int n_grid);

/* 自检入口：RS errata 编解码随机回归（trials 次），全部通过返回 0 */
int vdd_selftest_rs(int trials);

#ifdef __cplusplus
}
#endif
#endif
