#!/usr/bin/env python3
"""
将公开权重导出为 FP16 TensorRT 引擎。
Export the released weights to an FP16 TensorRT engine.
"""

import argparse
import os
import time
from pathlib import Path

from ultralytics import YOLO

DEFAULT_PT_PATH = Path(__file__).resolve().with_name("best_20260715.pt")
IMGSZ = 2464


def parse_args():
    parser = argparse.ArgumentParser(description="将实验 YOLO 权重导出为 TensorRT engine")
    parser.add_argument(
        "--weights",
        type=Path,
        default=DEFAULT_PT_PATH,
        help="输入 PT 权重路径（默认: scripts/yolo/best_20260715.pt）",
    )
    parser.add_argument("--device", type=int, default=0, help="CUDA 设备编号")
    return parser.parse_args()


def main():
    args = parse_args()
    weights_path = args.weights.expanduser().resolve()

    print('=' * 80)
    print('TensorRT 导出')
    print('=' * 80)
    print(f'输入:        {weights_path}')
    print(f'精度:        FP16')
    print(f'分辨率:      {IMGSZ}x{IMGSZ}')
    print(f'动态形状:    否')
    print()

    if not weights_path.is_file():
        raise FileNotFoundError(weights_path)

    model = YOLO(str(weights_path))
    print(f'task:  {model.task}')
    print(f'names: {model.names}')
    print()

    t0 = time.perf_counter()
    engine_path = model.export(
        format='engine',
        half=True,
        imgsz=IMGSZ,
        device=args.device,
        simplify=True,
        dynamic=False,
        verbose=False,
    )
    dt = time.perf_counter() - t0
    print()
    print(f'导出耗时: {dt:.1f}s')
    print(f'engine 文件: {engine_path}')
    print(f'大小:        {os.path.getsize(engine_path) / 1024 / 1024:.2f} MB')


if __name__ == '__main__':
    main()
