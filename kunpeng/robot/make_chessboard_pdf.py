#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_chessboard_pdf.py —— 生成「可打印的标定棋盘」PDF（A4，尺寸精确到毫米）

为什么要有这个脚本（而不是随手找一张棋盘图）：
  标定里 `square_m` 是**唯一的尺度基准**，它错 1%，25cm 处的深度就错 2.5mm。
  网上下的棋盘图不知道边长、也不知道出版社会不会缩放，等于把一个未知数塞进整条链路。
  这个脚本自己写 PDF：格子边长以毫米为单位精确给定（PDF 内部单位 1pt = 1/72 英寸，
  全是浮点数，不存在取整丢精度），另外印一条 100.0mm 校尺，打完用真尺子一量就知道
  打印机有没有偷偷缩放。

概念（大白话 / 规范定义）：
  大白话：PDF 里的坐标是「点」（pt），1 英寸 = 72 点，1 毫米 = 72/25.4 = 2.8346 点。
          把毫米乘这个数，画出来的方块在纸上就是准的毫米数。
  规范定义：PDF 页面坐标系原点在左下角，单位为用户空间单位（默认 1/72 inch）；
          ISO 216 A4 = 210 x 297 mm = 595.276 x 841.890 pt。矩形算子 `re` + `f`（fill）。

用法：
  python3 make_chessboard_pdf.py                     # 默认 8.2mm 格子 -> 82.0 x 57.4mm 板
  python3 make_chessboard_pdf.py --square 0.010      # 改用 10mm 格子（做对照实验时用）
  python3 make_chessboard_pdf.py --verify out.pdf    # 只校验已生成的 PDF

生成的 PDF 有两个用途，缺一不可：
  1) 棋盘本体：10 x 7 格，内角点 9 x 6（和 eye_check.py / handeye_calib.py 的 --cols 9 --rows 6 对齐）
  2) 100mm 校尺：打印后量它，判断打印机是否按 100% 输出

作者备注（踩坑记录）：
  - 别在 PDF 里写字号很小的中文：base-14 字体只保证拉丁字符，中文要么缺字要么乱码，
    要中文就得嵌字体（体积大、还容易踩 CID 编码坑）。所以本文件正文全 ASCII，
    中文说明放在 README 和对话里。
  - 别给棋盘加外框线：棋盘黑白区的边界由格子自己的边决定，外面再描一圈线会让
    findChessboardCorners 在边界处多出一层对比度，亚像素角点会被拉歪。
"""

import argparse
import os
import sys

# 1 mm = 72/25.4 pt。写成常量而不是每次乘，避免手误。
MM = 72.0 / 25.4


def mm(v):
    """毫米 -> PDF 用户单位（pt）。"""
    return float(v) * MM


def _esc(text):
    """PDF 字符串转义：只处理反斜杠与圆括号，其余 ASCII 原样。"""
    return text.replace("\\", r"\\").replace("(", r"\(").replace(")", r"\)")


# Helvetica 的字符宽度表（单位 1/1000 em，取自 Adobe 标准 AFM）。
# 为什么需要它：PDF 是"绝对定位"的，一行字多宽程序并不知道；
# 不给宽度就没法在生成阶段发现"文字被打印机右边距裁掉"这种问题。
_HELV = {
    " ": 278, "!": 278, '"': 355, "#": 556, "$": 556, "%": 889, "&": 667,
    "'": 191, "(": 333, ")": 333, "*": 389, "+": 584, ",": 278, "-": 333,
    ".": 278, "/": 278, ":": 278, ";": 278, "<": 584, "=": 584, ">": 584,
    "?": 556, "@": 1015, "[": 278, "\\": 278, "]": 278, "^": 469, "_": 556,
    "`": 333, "{": 334, "|": 260, "}": 334, "~": 584,
    "A": 667, "B": 667, "C": 722, "D": 722, "E": 667, "F": 611, "G": 778,
    "H": 722, "I": 278, "J": 500, "K": 667, "L": 556, "M": 833, "N": 722,
    "O": 778, "P": 667, "Q": 778, "R": 722, "S": 667, "T": 611, "U": 722,
    "V": 667, "W": 944, "X": 667, "Y": 667, "Z": 611,
    "a": 556, "b": 556, "c": 500, "d": 556, "e": 556, "f": 278, "g": 556,
    "h": 556, "i": 222, "j": 222, "k": 500, "l": 222, "m": 833, "n": 556,
    "o": 556, "p": 556, "q": 556, "r": 333, "s": 500, "t": 278, "u": 556,
    "v": 500, "w": 722, "x": 500, "y": 500, "z": 500,
}


def text_width_pt(s, size):
    """估算一行 Helvetica 文字的宽度（pt）。"""
    total = 0
    for ch in s:
        total += _HELV.get(ch, 556)      # 未知字符按数字宽度兜底
    return total / 1000.0 * size


def build_content(cols, rows, sq_mm, page_w_mm, page_h_mm):
    """
    生成页面内容流（一串 PDF 绘图算子）。

    坐标约定：
      - PDF 原点在左下角，y 向上。
      - 棋盘「左上角第一个格子是黑的」——这是 OpenCV 官方棋盘图的惯例，
        但 findChessboardCorners 本身不关心谁黑谁白，所以怎么起头都能检测。
    """
    sq = mm(sq_mm)
    bw = cols * sq                    # 板宽（pt）
    bh = rows * sq                    # 板高（pt）
    bx = (mm(page_w_mm) - bw) / 2.0   # 水平居中
    by = mm(200.0)                    # 板下沿距页面底边 200mm（上方留标题、下方留说明与校尺）

    ops = []
    ops.append("q")                   # 保存图形状态
    ops.append("1 J 1 j")             # 圆角线帽、圆角连接，线条端点更干净

    # ---------- ① 黑格 ----------
    ops.append("0 g")                 # 纯黑填充
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:      # 左上角 (0,0) 为黑
                x = bx + c * sq
                y = by + (rows - 1 - r) * sq   # r=0 是最上面一行，所以 y 要翻过来
                ops.append("%.4f %.4f %.4f %.4f re f" % (x, y, sq, sq))

    # ---------- ② 板心标记（四条短刻度，画在板外，绝不压住格子）----------
    cx = bx + bw / 2.0
    cy = by + bh / 2.0
    ops.append("0 G")                 # 黑色描边
    ops.append("%.3f w" % mm(0.25))
    tick_in, tick_out = mm(0.6), mm(3.5)      # 距板边 0.6mm 起，长 2.9mm
    segs = [
        (bx - tick_out, cy, bx - tick_in, cy),                    # 左
        (bx + bw + tick_in, cy, bx + bw + tick_out, cy),          # 右
        (cx, by - tick_out, cx, by - tick_in),                    # 下
        (cx, by + bh + tick_in, cx, by + bh + tick_out),          # 上
    ]
    for x1, y1, x2, y2 in segs:
        ops.append("%.4f %.4f m %.4f %.4f l S" % (x1, y1, x2, y2))

    # ---------- ③ 裁切线（板外 5mm 的虚线框）----------
    ops.append("0.55 G")
    ops.append("%.3f w" % mm(0.15))
    ops.append("[2 2] 0 d")           # 虚线：实 2pt 空 2pt
    pad = mm(5.0)
    ops.append("%.4f %.4f %.4f %.4f re S" % (bx - pad, by - pad, bw + 2 * pad, bh + 2 * pad))
    ops.append("[] 0 d")              # 恢复实线

    # ---------- ④ 100mm 校尺 ----------
    bar_x0 = (mm(page_w_mm) - mm(100.0)) / 2.0
    bar_y = mm(163.0)
    bar_len = mm(100.0)
    ops.append("0 G")
    ops.append("%.3f w" % mm(0.35))
    ops.append("%.4f %.4f m %.4f %.4f l S" % (bar_x0, bar_y, bar_x0 + bar_len, bar_y))
    # 两端长刻度、中间短刻度
    for frac, h in ((0.0, 5.0), (0.5, 3.0), (1.0, 5.0)):
        x = bar_x0 + bar_len * frac
        ops.append("%.3f w" % mm(0.35))
        ops.append("%.4f %.4f m %.4f %.4f l S" % (x, bar_y, x, bar_y + mm(h)))

    # ---------- ⑤ 文字（全部 ASCII，Helvetica）----------
    warn = []

    def text(x_mm, y_mm, size, s):
        w_pt = text_width_pt(s, size)
        if x_mm * MM + w_pt > mm(page_w_mm - 12.0):
            warn.append("文字越过右侧安全边距 %.1fmm：%s" % (page_w_mm - 12.0, s))
        ops.append("BT /F1 %.2f Tf 1 0 0 1 %.3f %.3f Tm (%s) Tj ET"
                   % (size, mm(x_mm), mm(y_mm), _esc(s)))

    left = 55.0                                   # 文字左边距（mm）
    text(left, 285.0, 12.0, "CALIBRATION CHESSBOARD   9 x 6 INNER CORNERS")
    text(left, 278.5, 10.0, "squares %d x %d   edge = %.2f mm   board = %.1f x %.1f mm"
         % (cols, rows, sq_mm, cols * sq_mm, rows * sq_mm))
    text(left, 272.0, 9.0,  "PRINT AT 100% / ACTUAL SIZE on A4.")
    text(left, 266.5, 9.0,  "Do NOT use 'Fit to page' / 'Shrink to fit' / 'Scale to paper'.")

    text(left, 196.0, 9.0,  "CHECK AFTER PRINTING:")
    text(left, 190.0, 9.0,  "  10 squares across (board width) must be %.1f mm" % (cols * sq_mm))
    text(left, 184.0, 9.0,  "  the 100 mm bar printed below must measure 100.0 mm")
    text(left, 177.5, 9.0,  "CENTER: the 4 short ticks outside the border mark the exact board centre.")
    text(left, 171.5, 9.0,  "Cut along the dashed line (5 mm margin). Plain matte paper only.")

    text(left, 154.0, 8.0,  "0")
    text(left + 47.0, 154.0, 8.0, "50.0")
    text(left + 91.0, 154.0, 8.0, "100.0 mm")

    text(left, 148.0, 8.5,  "1% scale error -> 1% depth error (about 2.5 mm at 25 cm).")
    text(left, 142.0, 8.5,  "If the printed board is not exact, set square_m = measured / 10000")
    text(left, 136.0, 8.5,  "in camera_intrinsics.json.  Downstream: --cols 9 --rows 6")

    ops.append("Q")
    return "\n".join(ops), warn


def write_pdf(path, content, page_w_mm, page_h_mm):
    """手写一个最小 PDF（单页）。不依赖任何第三方库，避免装包。"""
    data = content.encode("latin-1")
    page_w, page_h = mm(page_w_mm), mm(page_h_mm)

    objs = []
    objs.append(b"<< /Type /Catalog /Pages 2 0 R >>")
    objs.append(b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>")
    objs.append(("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.4f %.4f] "
                 "/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>"
                 % (page_w, page_h)).encode("latin-1"))
    objs.append(b"<< /Length " + str(len(data)).encode("ascii") + b" >>\nstream\n"
                + data + b"\nendstream")
    objs.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>")

    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += ("%d 0 obj\n" % i).encode("latin-1") + body + b"\nendobj\n"

    xref_pos = len(out)
    total = len(objs) + 1
    out += ("xref\n0 %d\n" % total).encode("latin-1")
    out += b"0000000000 65535 f \n"
    for off in offsets:
        out += ("%010d 00000 n \n" % off).encode("latin-1")
    out += ("trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
            % (total, xref_pos)).encode("latin-1")

    with open(path, "wb") as f:
        f.write(bytes(out))
    return len(out)


def verify(path, cols, rows, sq_mm):
    """
    独立校验：用 pypdf 重新读回来，检查
      ① 页面尺寸是不是 A4（595.28 x 841.89 pt）
      ② 黑格是不是正好 35 个（10x7 的一半）
      ③ 每个黑格的边长是不是 square 的精确值（取第一个检查）
      ④ 页面上能不能解析出文字
    只读不写，和生成逻辑互相独立，所以能抓到"生成时算错"这类问题。
    """
    from pypdf import PdfReader
    r = PdfReader(path)
    box = r.pages[0].mediabox
    w = float(box.width)
    h = float(box.height)
    print("页面尺寸   : %.3f x %.3f pt  (A4 应为 595.276 x 841.890)" % (w, h))
    ok = abs(w - 595.276) < 0.05 and abs(h - 841.890) < 0.05

    raw = r.pages[0].get_contents().get_data().decode("latin-1")
    rects = [ln for ln in raw.splitlines() if ln.rstrip().endswith("re f")]
    print("黑格数量   : %d  (应为 %d = %d x %d / 2)"
          % (len(rects), cols * rows // 2, cols, rows))
    ok = ok and len(rects) == cols * rows // 2

    if rects:
        parts = rects[0].split()
        side = float(parts[2])
        want = mm(sq_mm)
        print("黑格边长   : %.4f pt = %.4f mm  (应为 %.4f mm)"
              % (side, side / MM, sq_mm))
        ok = ok and abs(side - want) < 1e-3

    txt = r.pages[0].extract_text() or ""
    print("文字可解析 : %s  (%d 字符)" % ("是" if txt.strip() else "否", len(txt)))
    ok = ok and bool(txt.strip())
    print("总体校验   : %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="生成可打印标定棋盘 PDF（尺寸精确到毫米）")
    ap.add_argument("--cols", type=int, default=10, help="格子列数（内角点 9），默认 10")
    ap.add_argument("--rows", type=int, default=7, help="格子行数（内角点 6），默认 7")
    ap.add_argument("--square", type=float, default=0.0082,
                    help="格子边长，单位米；默认 0.0082（与 camera_intrinsics.json 的 square_m 一致）")
    ap.add_argument("--page-w", type=float, default=210.0, help="纸张宽 mm，默认 A4 210")
    ap.add_argument("--page-h", type=float, default=297.0, help="纸张高 mm，默认 A4 297")
    ap.add_argument("--out", default="", help="输出 PDF 路径")
    ap.add_argument("--verify", default="", help="只校验一个已有 PDF，不生成")
    args = ap.parse_args()

    sq_mm = args.square * 1000.0

    if args.verify:
        return verify(args.verify, args.cols, args.rows, sq_mm)

    out = args.out
    if not out:
        here = os.path.dirname(os.path.abspath(__file__))
        out = os.path.join(here, "chessboard_%dx%d_%.1fmm.pdf" % (args.cols, args.rows, sq_mm))

    content, warn = build_content(args.cols, args.rows, sq_mm, args.page_w, args.page_h)
    n = write_pdf(out, content, args.page_w, args.page_h)

    print("棋盘        : %d x %d 格（内角点 %d x %d）" % (args.cols, args.rows, args.cols - 1, args.rows - 1))
    print("格子边长    : %.2f mm" % sq_mm)
    print("板面尺寸    : %.2f x %.2f mm" % (args.cols * sq_mm, args.rows * sq_mm))
    print("输出        : %s  (%.1f KB)" % (out, n / 1024.0))
    if warn:
        print("")
        for w in warn:
            print("⚠ 版式警告：%s" % w)
    print("")
    print("打印注意    : A4、100%%/实际大小，关掉「适应页面」；打完量 10 格总宽必须是 %.1f mm" % (args.cols * sq_mm))
    return 0


if __name__ == "__main__":
    sys.exit(main())
