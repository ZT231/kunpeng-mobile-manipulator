#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kunpeng/llm —— 意图解析层（阶段 2）。

这个空文件只有一个作用：让 llm/ 成为一个 Python 包（package）。
`python -m unittest discover -s <目录>` 在 Python 3.11 之后要求起始目录
可导入，目录里没有 __init__.py 会报
`ImportError: Start directory is not importable`。
大白话：给这个文件夹挂个「我是正经包」的牌子，测试发现器才肯进来找考卷。
规范定义：PEP 420 之前，含 __init__.py 的目录才是 regular package；
         unittest.TestLoader.discover 依赖该目录可导入来推导模块名。
"""
