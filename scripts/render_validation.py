#!/usr/bin/env python3
"""将当前源码检查结果整理为本地文字报告，不嵌入图片或历史验收结论。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path("dist/validation"))
    args = parser.parse_args()
    directory = args.directory
    directory.mkdir(parents=True, exist_ok=True)
    lines = ["# 源码检查记录", "", "本记录不代替在线 YOLO、完整飞行或实机验收。", ""]
    privacy_path = directory / "privacy.json"
    if privacy_path.exists():
        privacy = json.loads(privacy_path.read_text())
        lines.append(f"公开文件隐私检查：扫描 {privacy['files_scanned']} 个文件，发现 {len(privacy['findings'])} 项。")
    else:
        lines.append("公开文件隐私检查：未运行。")
    tests_path = directory / "tests.xml"
    if tests_path.exists():
        root = ET.parse(tests_path).getroot()
        suites = [root] if root.tag == "testsuite" else list(root)
        counts = {key: sum(int(suite.get(key, 0)) for suite in suites)
                  for key in ("tests", "failures", "errors", "skipped")}
        lines.extend(["", f"Python 测试：{counts['tests']} 项，失败 {counts['failures']}，错误 {counts['errors']}，跳过 {counts['skipped']}。"])
    else:
        lines.extend(["", "Python 测试：未运行。"])
    lines.extend(["", "详细结果见同目录 privacy.json 与 tests.xml。资源与 SDF 检查以 verify_release.sh 的执行结果为准。", ""])
    (directory / "README.md").write_text("\n".join(lines), encoding="utf-8")
    print("已生成源码检查文字报告")


if __name__ == "__main__":
    main()
