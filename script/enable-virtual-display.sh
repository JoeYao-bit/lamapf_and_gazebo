#!/bin/bash

# -------------------------------
# 自适应虚拟显示脚本
# 有物理显示器时不启用虚拟屏幕
# 无物理显示器时启用虚拟屏幕，分辨率1280x720
# -------------------------------

# 检测物理显示器
CONNECTED=$(xrandr | grep " connected" | wc -l)

if [ "$CONNECTED" -gt 0 ]; then
    echo "Physical monitor detected, skipping virtual display."
    exit 0
fi

echo "No physical monitor detected, starting virtual display."

# -------------------------------
# 1280x720 @ 60Hz 模式
# -------------------------------
# 使用 cvt 生成的 Modeline:
# cvt 1280 720 60
# Modeline "1280x720_60.00" 74.50 1280 1344 1472 1664 720 723 728 748 -hsync +vsync

xrandr --newmode "1280x720_60.00" 74.50 1280 1344 1472 1664 720 723 728 748 -hsync +vsync
xrandr --addmode VIRTUAL1 1280x720_60.00
xrandr --output VIRTUAL1 --mode 1280x720_60.00

echo "Virtual display 1280x720 started."
