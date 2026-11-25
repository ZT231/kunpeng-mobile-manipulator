#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
源码编码规范检查器 —— 防止 Keil 中文注释乱码复发。

背景
    Keil uVision 打开"UTF-8 无 BOM"的文件时，会按 ANSI(GB2312) 解码，
    中文注释显示为乱码；文件带 UTF-8 BOM 则能被自动识别，显示正常。

规则
    1) Core/ 下参与 Keil 编译的 .c/.h：只要含非 ASCII 字符，必须为 UTF-8 with BOM；
    2) 仓库内其它源码/文档：必须为合法 UTF-8（BOM 可有可无，但不允许 GBK/ANSI）；
    3) 任何"非法 UTF-8"一律判错 —— 那说明文件被存成了 ANSI/GB2312。

用法
    python tools/check_encoding.py          # 只检查
    python tools/check_encoding.py --fix    # 顺带给 Core/ 的 .c/.h 自动补 BOM

退出码
    0 = 全部合规；1 = 存在违规（可挂到提交前钩子 / CI）
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOM = b"\xef\xbb\xbf"

# 必须带 BOM 的目录（Keil 参与编译）
KEIL_DIRS = ("Core",)
# 只要求"合法 UTF-8"的目录
UTF8_DIRS = ("esp32", "protocol_engine_cpp", "docs", "kunpeng", "iot", "tools")
# 参与检查的扩展名
EXTS = {".c", ".h", ".cpp", ".hpp", ".cc", ".py", ".md", ".txt"}
# 跳过目录：第三方库、Keil 生成物、版本库
SKIP = {"Drivers", "Middlewares", "MDK-ARM", ".git", "build", ".vscode", "__pycache__"}


def iter_files(base: Path):
    """按扩展名遍历，剪掉 SKIP 目录（避免爬 .git）。"""
    for dirpath, dirnames, filenames in os.walk(base):
        dirnames[:] = [d for d in dirnames if d not in SKIP]
        for name in filenames:
            p = Path(dirpath) / name
            if p.suffix.lower() in EXTS:
                yield p


def inspect(path: Path, need_bom: bool) -> tuple[str | None, bool]:
    """返回 (问题描述或 None, 是否含非 ASCII 字节)。"""
    data = path.read_bytes()
    non_ascii = any(b > 0x7F for b in data)
    has_bom = data.startswith(BOM)

    try:
        data.decode("utf-8")          # 严格模式，非法字节会抛 UnicodeDecodeError
    except UnicodeDecodeError:
        return "不是合法 UTF-8（疑似被存成 GBK/ANSI）", non_ascii

    if need_bom and non_ascii and not has_bom:
        return "含中文但缺 UTF-8 BOM（Keil 中会显示乱码）", non_ascii

    return None, non_ascii


def add_bom(path: Path) -> None:
    path.write_bytes(BOM + path.read_bytes())


def main() -> int:
    do_fix = "--fix" in sys.argv[1:]
    errors: list[str] = []
    fixed: list[str] = []
    checked = 0

    # 1) Keil 源码：要求 UTF-8 with BOM
    for d in KEIL_DIRS:
        base = ROOT / d
        if not base.is_dir():
            continue
        for f in iter_files(base):
            checked += 1
            problem, non_ascii = inspect(f, need_bom=True)
            if problem is None:
                continue
            if do_fix and non_ascii and "缺 UTF-8 BOM" in problem:
                add_bom(f)
                fixed.append(str(f.relative_to(ROOT)))
            else:
                errors.append(f"{f.relative_to(ROOT)}: {problem}")

    # 2) 其它目录：只要求合法 UTF-8
    for d in UTF8_DIRS:
        base = ROOT / d
        if not base.is_dir():
            continue
        for f in iter_files(base):
            checked += 1
            problem, _ = inspect(f, need_bom=False)
            if problem:
                errors.append(f"{f.relative_to(ROOT)}: {problem}")

    # 3) 仓库根目录的散文件（README.md 等）
    for f in ROOT.iterdir():
        if f.is_file() and f.suffix.lower() in EXTS:
            checked += 1
            problem, _ = inspect(f, need_bom=False)
            if problem:
                errors.append(f"{f.name}: {problem}")

    print(f"已检查文件: {checked}")
    if fixed:
        print(f"已自动补 BOM: {len(fixed)}")
        for x in fixed:
            print("  + " + x)

    if errors:
        print(f"发现 {len(errors)} 处问题:")
        for e in errors:
            print("  - " + e)
        return 1

    print("编码规范检查通过（Keil 源码均为 UTF-8 with BOM，其余为合法 UTF-8）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
