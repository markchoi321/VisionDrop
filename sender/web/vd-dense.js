/*
 * VisionDrop 密集彩色 L1（VD-D1）编码器 —— 纯 ES5
 *
 * 规约见 protocol/dense/DENSE.md，语义与 protocol/dense/dense_refimpl.py
 * 逐位一致，由 test/run-dense-vectors.js 对 vectors/ 全量验收。
 *
 * 8 色调色板取 RGB 立方体八角（借鉴 JAB Code），每格 3 bit：
 * bit2=R bit1=G bit0=B。载荷是一个完整的 L2 帧，L2-L5 协议不变。
 * 4 色降级档只用 {黑, 青, 品红, 黄} 四角，每格 2 bit，最小色距是面对角线；
 * 调制方式写在头部版本字节低 2 位（0 = 8 色，1 = 4 色）。
 *
 * 零依赖：GF(256) 与 RS 编码内置于本文件。
 */
(function (root, factory) {
    if (typeof module === "object" && module.exports) {
        module.exports = factory();
    } else {
        root.VDD = factory();
    }
}(typeof self !== "undefined" ? self : this, function () {
    "use strict";

    /* ------------------------------------------------ GF(256) 与 RS 编码 */

    var GF_EXP = new Array(512), GF_LOG = new Array(256);
    (function () {
        var x = 1;
        for (var i = 0; i < 255; i++) {
            GF_EXP[i] = x;
            GF_LOG[x] = i;
            x <<= 1;
            if (x & 0x100) { x ^= 0x11D; }
        }
        for (var j = 255; j < 512; j++) { GF_EXP[j] = GF_EXP[j - 255]; }
    })();

    function gfMul(a, b) {
        if (a === 0 || b === 0) { return 0; }
        return GF_EXP[GF_LOG[a] + GF_LOG[b]];
    }

    /* 生成多项式缓存：degree -> 系数数组（首一，最高次在前）
       g(x) = (x + a^0)(x + a^1)…(x + a^(degree-1)) */
    var GEN_CACHE = {};
    function genPoly(degree) {
        if (GEN_CACHE[degree]) { return GEN_CACHE[degree]; }
        var poly = [1];
        for (var d = 0; d < degree; d++) {
            var next = new Array(poly.length + 1);
            var i;
            for (i = 0; i < next.length; i++) { next[i] = 0; }
            for (i = 0; i < poly.length; i++) {
                next[i] ^= poly[i];                        /* 乘 x 项 */
                next[i + 1] ^= gfMul(poly[i], GF_EXP[d]);  /* 乘 a^d 项 */
            }
            poly = next;
        }
        GEN_CACHE[degree] = poly;
        return poly;
    }

    /* RS 纠错码字：data 为 Uint8Array，返回 Uint8Array(degree) */
    function rsEncode(data, degree) {
        var gen = genPoly(degree);
        var res = new Uint8Array(degree);
        for (var i = 0; i < data.length; i++) {
            var factor = data[i] ^ res[0];
            /* 左移一格 */
            for (var j = 0; j < degree - 1; j++) { res[j] = res[j + 1]; }
            res[degree - 1] = 0;
            if (factor !== 0) {
                var lf = GF_LOG[factor];
                for (var k = 0; k < degree; k++) {
                    /* gen[0] 恒为 1（首一），余数只需异或 gen[k+1] 的贡献 */
                    var g = gen[k + 1];
                    if (g !== 0) { res[k] ^= GF_EXP[(GF_LOG[g] + lf) % 255]; }
                }
            }
        }
        return res;
    }

    var GRID_SIZES = [64, 80, 96, 112, 128];
    var FINDER = 7;
    var CORNER = 8;                  /* 角部功能块：定位图形 + 分隔带 */
    var CAL_LEN = 8;
    var HEADER_CELLS = 120;          /* 40 bit x 3 重复 */
    var RS_PARITY = 32;
    var RS_DATA_MAX = 223;
    var MAGIC = 0xD5;
    var VERSION = 1;

    /* 调色板的 CSS 颜色，下标 = 格值 */
    var PALETTE_CSS = ["#000000", "#0000ff", "#00ff00", "#00ffff",
                       "#ff0000", "#ff00ff", "#ffff00", "#ffffff"];

    /* 4 色档：2 bit 值 -> 格值（黑、青、品红、黄），与 dense_refimpl.PALETTE4 一致 */
    var PALETTE4 = [0, 3, 5, 6];
    var MOD_4 = 1;

    /* 每格位数；colors 缺省为 8 */
    function bitsPerCell(colors) {
        if (colors === undefined || colors === 8) { return 3; }
        if (colors === 4) { return 2; }
        throw new Error("色数只能是 8 或 4: " + colors);
    }

    /* ------------------------------------------------ 结构 */

    function inCornerBlock(r, c, N) {
        return (r < CORNER && c < CORNER) || (r < CORNER && c >= N - CORNER) ||
               (r >= N - CORNER && c < CORNER) ||
               (r >= N - CORNER && c >= N - CORNER);
    }

    function calStrips(N) {
        return [[CORNER, 1], [CORNER, N - 1 - CAL_LEN],
                [N - 1 - CORNER, 1], [N - 1 - CORNER, N - 1 - CAL_LEN]];
    }

    /*
     * 自由格行优先列表，元素为 r*N+c。
     *
     * 按 N 缓存。这个表只由 N 决定，与载荷无关，但 encode() 每帧要用两次
     * （capacity 里一次、排布数据时一次）。N=128 时它要扫 16384 格再 push
     * 出一万五千个下标，每帧重算两遍在 60 fps 下是纯浪费。
     * 返回的是共享只读对象，调用方不得修改。
     */
    var FREE_CACHE = {};
    function freeCells(N) {
        if (FREE_CACHE[N]) { return FREE_CACHE[N]; }
        var fn = new Uint8Array(N * N);
        var r, c, i;
        for (r = 0; r < N; r++) {
            for (c = 0; c < N; c++) {
                if (inCornerBlock(r, c, N)) { fn[r * N + c] = 1; }
            }
        }
        for (i = 0; i < N; i++) {
            fn[i] = 1; fn[(N - 1) * N + i] = 1;
            fn[i * N] = 1; fn[i * N + N - 1] = 1;
        }
        var strips = calStrips(N);
        for (i = 0; i < 4; i++) {
            for (c = 0; c < CAL_LEN; c++) {
                fn[strips[i][0] * N + strips[i][1] + c] = 1;
            }
        }
        var cnt = 0;
        for (i = 0; i < N * N; i++) { if (!fn[i]) { cnt++; } }
        var out = new Int32Array(cnt);
        var p = 0;
        for (i = 0; i < N * N; i++) { if (!fn[i]) { out[p++] = i; } }
        FREE_CACHE[N] = out;
        return out;
    }

    /* 返回 {free, dataCells, capBytes, net}，按 N 与色数缓存，共享只读 */
    var CAP_CACHE = {};
    function capacity(N, colors) {
        var bpc = bitsPerCell(colors);
        var key = N + "/" + bpc;
        if (CAP_CACHE[key]) { return CAP_CACHE[key]; }
        var free = freeCells(N).length;
        var dataCells = free - HEADER_CELLS;
        var capBytes = Math.floor(dataCells * bpc / 8);
        var net = 0;
        for (var p = capBytes; p > 0; p--) {
            if (p + RS_PARITY * Math.ceil(p / RS_DATA_MAX) <= capBytes) {
                net = p;
                break;
            }
        }
        var r = { free: free, dataCells: dataCells, capBytes: capBytes, net: net };
        CAP_CACHE[key] = r;
        return r;
    }

    /* ------------------------------------------------ 头部 */

    function crc8(bytes, len) {
        var c = 0;
        for (var i = 0; i < len; i++) {
            c ^= bytes[i];
            for (var b = 0; b < 8; b++) {
                c = (c & 0x80) ? (((c << 1) ^ 0x07) & 0xFF) : ((c << 1) & 0xFF);
            }
        }
        return c;
    }

    function buildHeader(payloadLen, colors) {
        var h = new Uint8Array(5);
        h[0] = MAGIC;
        h[1] = (VERSION << 4) | (bitsPerCell(colors) === 2 ? MOD_4 : 0);
        h[2] = (payloadLen >>> 8) & 0xFF;
        h[3] = payloadLen & 0xFF;
        h[4] = crc8(h, 4);
        return h;
    }

    /* ------------------------------------------------ RS 分块 + 交织 */

    function rsBlocks(payload) {
        var P = payload.length;
        var nb = Math.ceil(P / RS_DATA_MAX);
        var base = Math.floor(P / nb);
        var rem = P % nb;
        var blocks = [], ecs = [];
        var off = 0, i, j;
        for (i = 0; i < nb; i++) {
            var ln = base + (i >= nb - rem ? 1 : 0);
            var blk = payload.subarray(off, off + ln);
            off += ln;
            blocks.push(blk);
            ecs.push(rsEncode(blk, RS_PARITY));
        }
        var out = new Uint8Array(P + nb * RS_PARITY);
        var pos = 0;
        var maxLen = base + (rem > 0 ? 1 : 0);
        for (j = 0; j < maxLen; j++) {
            for (i = 0; i < nb; i++) {
                if (j < blocks[i].length) { out[pos++] = blocks[i][j]; }
            }
        }
        for (j = 0; j < RS_PARITY; j++) {
            for (i = 0; i < nb; i++) { out[pos++] = ecs[i][j]; }
        }
        return out;
    }

    /* ------------------------------------------------ 编码 */

    var RING = ["1111111", "1000001", "1011101", "1011101",
                "1011101", "1000001", "1111111"];

    /*
     * 固定结构模板：白底 + 定位图形 + 边框环 + 定标条。这些格子只由 N 决定，
     * 逐帧重画一遍毫无意义，缓存下来每帧拷贝一份即可。
     */
    var TPL_CACHE = {};
    function staticTemplate(N) {
        if (TPL_CACHE[N]) { return TPL_CACHE[N]; }
        var cells = new Uint8Array(N * N);
        var i, r, c, dr, dc;
        for (i = 0; i < N * N; i++) { cells[i] = 7; }      /* 默认白 */

        /* 定位图形（右下角芯为红=4） */
        var corners = [[0, 0, 0], [0, N - FINDER, 0],
                       [N - FINDER, 0, 0], [N - FINDER, N - FINDER, 4]];
        for (i = 0; i < 4; i++) {
            var r0 = corners[i][0], c0 = corners[i][1], core = corners[i][2];
            for (dr = 0; dr < FINDER; dr++) {
                for (dc = 0; dc < FINDER; dc++) {
                    var dark = RING[dr].charAt(dc) === "1";
                    var v = dark ? 0 : 7;
                    if (dark && dr >= 2 && dr <= 4 && dc >= 2 && dc <= 4) {
                        v = core;
                    }
                    cells[(r0 + dr) * N + (c0 + dc)] = v;
                }
            }
        }
        /* 边框环（角部块之外） */
        for (i = 0; i < N; i++) {
            var edges = [[0, i], [N - 1, i], [i, 0], [i, N - 1]];
            for (var e = 0; e < 4; e++) {
                r = edges[e][0]; c = edges[e][1];
                if (!inCornerBlock(r, c, N)) {
                    cells[r * N + c] = (r + c) % 2 === 0 ? 0 : 7;
                }
            }
        }
        /* 定标条 */
        var strips = calStrips(N);
        for (i = 0; i < 4; i++) {
            for (c = 0; c < CAL_LEN; c++) {
                cells[strips[i][0] * N + strips[i][1] + c] = c;
            }
        }
        TPL_CACHE[N] = cells;
        return cells;
    }

    /* payload(Uint8Array) -> {n: N, cells: Uint8Array(N*N)，值 0-7}。
       colors 取 8（缺省）或 4；4 色档的数据格只出现 PALETTE4 里的格值 */
    function encode(payload, N, colors) {
        var bpc = bitsPerCell(colors);
        var cap = capacity(N, colors);
        if (payload.length > cap.net) {
            throw new Error("载荷超出容量: N=" + N + " len=" + payload.length +
                " net=" + cap.net);
        }
        var cells = new Uint8Array(staticTemplate(N));    /* 固定结构一次拷贝到位 */
        var i;

        /* 头部：40 bit x 3 副本，白=0 黑=1 */
        var free = freeCells(N);
        var head = buildHeader(payload.length, colors);
        var bits = [];
        for (i = 0; i < 5; i++) {
            for (var b = 7; b >= 0; b--) { bits.push((head[i] >>> b) & 1); }
        }
        for (var k = 0; k < 3; k++) {
            for (var j = 0; j < 40; j++) {
                cells[free[j + 40 * k]] = bits[j] ? 0 : 7;
            }
        }
        /* 数据：交织流 MSB 在前，每格 bpc 位（8 色 3 bit，4 色 2 bit 再查表） */
        var stream = rsBlocks(payload);
        var totalBits = stream.length * 8;
        var bitpos = 0;
        for (i = HEADER_CELLS; i < free.length; i++) {
            var val = 0;
            for (var t = 0; t < bpc; t++) {
                var bit = 0;
                if (bitpos < totalBits) {
                    bit = (stream[bitpos >> 3] >>> (7 - (bitpos & 7))) & 1;
                }
                val = (val << 1) | bit;
                bitpos++;
            }
            cells[free[i]] = bpc === 2 ? PALETTE4[val] : val;
        }
        return { n: N, cells: cells };
    }

    return {
        GRID_SIZES: GRID_SIZES,
        PALETTE_CSS: PALETTE_CSS,
        PALETTE4: PALETTE4,
        QUIET: 2,
        capacity: capacity,
        rsEncode: rsEncode,
        buildHeader: buildHeader,
        rsBlocks: rsBlocks,
        encode: encode
    };
}));
