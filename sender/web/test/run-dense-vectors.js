#!/usr/bin/env node
/*
 * VD-D1 密集彩色 L1 向量测试：比对 protocol/dense/vectors/。
 * 运行：node sender/web/test/run-dense-vectors.js
 */
"use strict";

var fs = require("fs");
var path = require("path");

var VDP = require("../vd-protocol.js");
var VDD = require("../vd-dense.js");

var VEC = path.join(__dirname, "..", "..", "..", "protocol", "dense", "vectors");
var failures = 0, checks = 0;

function ok(cond, msg) {
    checks++;
    if (!cond) { failures++; console.log("失败: " + msg); }
}

function rawLines(name) {
    return fs.readFileSync(path.join(VEC, name), "utf8").split("\n")
        .filter(function (l) { return l.length > 0 && l.charAt(0) !== "#"; });
}

function hex(u8) {
    var s = "";
    for (var i = 0; i < u8.length; i++) {
        s += (u8[i] < 16 ? "0" : "") + u8[i].toString(16);
    }
    return s;
}

function fromHex(h) {
    var out = new Uint8Array(h.length / 2);
    for (var i = 0; i < out.length; i++) {
        out[i] = parseInt(h.substr(i * 2, 2), 16);
    }
    return out;
}

/* 1. 头部 */
rawLines("dense_header.txt").forEach(function (l) {
    var p = l.split(" ");
    ok(hex(VDD.buildHeader(+p[0])) === p[1], "dense_header len=" + p[0]);
});
console.log("dense_header.txt 通过");

/* 2. 容量 */
rawLines("dense_capacity.txt").forEach(function (l) {
    var p = l.split(" ").map(Number);
    var c = VDD.capacity(p[0]);
    ok(c.free === p[1] && c.dataCells === p[2] && c.capBytes === p[3] && c.net === p[4],
        "dense_capacity N=" + p[0] + " 实得 " + JSON.stringify(c));
});
console.log("dense_capacity.txt 通过");

/* 3. RS */
rawLines("dense_rs.txt").forEach(function (l) {
    var p = l.split(" ");
    var got = hex(VDD.rsEncode(fromHex(p[0]), 32));
    ok(got === p[1], "dense_rs len=" + p[0].length / 2);
});
console.log("dense_rs.txt 通过");

/* 4. 整符号 */
var symLines = rawLines("dense_symbol.txt");
for (var i = 0; i + 1 < symLines.length; i += 2) {
    var p = symLines[i].split(" ");
    var N = +p[0];
    var payload = fromHex(p[2]);
    ok(payload.length === +p[1], "dense_symbol 载荷长度");
    var sym = VDD.encode(payload, N);
    var got = "";
    for (var j = 0; j < sym.cells.length; j++) { got += sym.cells[j]; }
    ok(got === symLines[i + 1], "dense_symbol N=" + N + " 格值不一致");
}
console.log("dense_symbol.txt 通过");

console.log("\n共 " + checks + " 项检查，失败 " + failures + " 项");
process.exit(failures === 0 ? 0 : 1);
