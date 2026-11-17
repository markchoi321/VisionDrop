#!/usr/bin/env python3
"""生成 VD-D1 金图像测试资产（供 iOS 接收端回归）。

金图像是「真实 L2 帧 -> VD-D1 符号 -> 渲染 -> 光度/几何失真」的产物，
权威副本写在 protocol/dense/golden/，测试副本在
receiver/QrBinaryTests/vectors/dense/，由 Xcode 的 "Verify Protocol Vectors"
构建阶段校验两份一致。

用法：
    python3 protocol/dense/gen_golden.py          写权威副本
    python3 protocol/dense/gen_golden.py --sync   同时同步到测试副本
"""

import hashlib
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import dense_refimpl as D
import refimpl as R

GOLDEN = os.path.join(HERE, "golden")
MIRROR = os.path.join(HERE, "..", "..", "receiver", "QrBinaryTests", "vectors", "dense")

# (名字, N, baseBlockId, 构造函数[, 色数])。构造函数接收 cells 与 N，返回图像。
# 色数缺省 8；4 色金图像的文件名带 _4c，清单格式不变（解码端从头部读色数）。
def _plain(kw):
    def f(cells, N):
        img = D.render(cells, N, scale=8)
        return D.distort(img, **kw) if kw else img
    return f


def _barrel(k1):
    """桶形/枕形镜头畸变：四角单应性表达不了的形变。"""
    def f(cells, N):
        return D.barrel(D.render(cells, N, scale=8), k1)
    return f


def _moire(period, depth, out_size, kw=None):
    """屏幕像素栅格 + 传感器点采样 = 真摩尔纹。"""
    def f(cells, N):
        img = D.screen_grid(D.render(cells, N, scale=10), period=period, depth=depth)
        img = D.resample_point(img, out_size, out_size)
        return D.distort(img, **kw) if kw else img
    return f


def _barrel_moire(k1, period, depth, out_size, kw):
    def f(cells, N):
        img = D.screen_grid(D.render(cells, N, scale=10), period=period, depth=depth)
        img = D.barrel(img, k1)
        img = D.resample_point(img, out_size, out_size)
        return D.distort(img, **kw)
    return f


def _xtalk(x):
    """通道串扰矩阵（行 = 输出通道）：绿基色漏进 R 最重，对应真机 R 通道读到 114。"""
    return [[1 - 0.2 * x, 0.9 * x, 0.2 * x],
            [0.25 * x, 1 - 0.3 * x, 0.3 * x],
            [0.15 * x, 0.45 * x, 1 - 0.2 * x]]


CASES = [
    ("g96_clean", 96, 0, _plain(None)),
    ("g96_combo", 96, 9, _plain(dict(gamma=(0.9, 1.15, 1.05), gain=(0.95, 1.02, 0.9),
                                     noise=8, warp=0.006, seed=7))),
    ("g96_noise", 96, 18, _plain(dict(noise=14, seed=11))),
    ("g64_dist", 64, 27, _plain(dict(gamma=(0.88, 1.1, 1.0), noise=8, warp=0.005, seed=13))),
    ("g128_dist", 128, 36, _plain(dict(gain=(0.92, 1.05, 0.97), noise=6, warp=0.004, seed=17))),
    # 以下三例针对镜头畸变与摩尔纹，未做几何加固前全部解不出
    ("g96_barrel", 96, 45, _barrel(-0.055)),
    ("g96_moire", 96, 54, _moire(10, 0.45, 350)),
    ("g96_bm", 96, 63, _barrel_moire(-0.05, 10, 0.4, 420,
                                     dict(gamma=(0.9, 1.12, 1.02), noise=6, seed=5))),
    # 4 色降级档。xtalk 一例：同一失真下 8 色符号八色最小间距约 13，
    # 解不出；选定四色仍有约 70 的间距，4 色符号零纠正解出
    ("g96_4c_dist", 96, 72, _plain(dict(gamma=(0.88, 1.1, 1.0), noise=8, warp=0.005,
                                        seed=13)), 4),
    ("g128_4c_xtalk", 128, 81, _plain(dict(xtalk=_xtalk(0.85), lift=100, gain=(0.95, 1.0, 0.9),
                                           noise=8, warp=0.004, seed=21)), 4),
]


def gen():
    os.makedirs(GOLDEN, exist_ok=True)
    content = R._det_bytes(20000, 2026)
    stream, _ = R.build_stream(content, "fixture.bin", allow_compress=False)
    T = R.T_MIN_LT
    K = -(-len(stream) // T)
    sid = R.session_id(hashlib.sha256(content).digest(), T, K, R.CODEC_LT)
    cdf = R.rs_cdf_q(K)
    src = R.split_source_blocks(stream, K, T)

    manifest = ["# VD-D1 金图像清单（gen_golden.py 生成）",
                "# 格式: <bmp文件名> <N> <期望 L2 帧 hex>"]
    for case in CASES:
        name, N, base, build = case[:4]
        colors = case[4] if len(case) > 4 else 8
        _, _, _, net = D.capacity(N, colors)
        m = (net - R.FRAME_HEADER_LEN) // T
        if m > 255:
            m = 255
        frame = R.encode_frame(src, K, T, R.CODEC_LT, False, sid, base, m, cdf)
        cells = D.encode_symbol(frame, N, colors)
        img = build(cells, N)
        D.write_bmp(os.path.join(GOLDEN, name + ".bmp"), img)
        manifest.append("%s.bmp %d %s" % (name, N, frame.hex()))
        print("  %s N=%d %d色 帧长 %d" % (name, N, colors, len(frame)))
    with open(os.path.join(GOLDEN, "manifest.txt"), "w") as f:
        f.write("\n".join(manifest) + "\n")
    print("权威副本已写入 %s" % GOLDEN)


def sync():
    if os.path.isdir(MIRROR):
        shutil.rmtree(MIRROR)
    shutil.copytree(GOLDEN, MIRROR)
    print("测试副本已同步到 %s" % os.path.normpath(MIRROR))


if __name__ == "__main__":
    gen()
    if "--sync" in sys.argv:
        sync()
