#!/bin/bash

# 项目根目录
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
# CMake 构建目录
BUILD_DIR="$ROOT_DIR/SYSU_Infantry/build/Debug"
ELF_PATH="$BUILD_DIR/SYSU_Infantry.elf"

echo "=== 开始编译 ==="
ninja -C "$BUILD_DIR"

if [ $? -ne 0 ]; then
  echo "编译失败!"
  exit 1
fi

echo "=== 开始烧录 ==="
sudo openocd -f "$ROOT_DIR/stm32f407.cfg" \
  -c "program $ELF_PATH verify reset exit"

if [ $? -eq 0 ]; then
  echo "=== 烧录完成 ==="
else
  echo "=== 烧录失败 ==="
  exit 1
fi
