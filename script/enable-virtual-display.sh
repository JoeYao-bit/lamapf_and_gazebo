#!/bin/bash

# -------------------------------
# 自适应虚拟显示脚本（1280x720）
# 有物理显示器时关闭虚拟屏幕
# 无物理显示器时启用虚拟屏幕
# 支持热插拔
# -------------------------------

# 虚拟屏幕名称
VIRTUAL_OUTPUT="VIRTUAL1"
VIRTUAL_MODE="1280x720_60.00"

# 检测物理显示器
PHYSICAL_COUNT=$(xrandr | grep " connected" | grep -v "$VIRTUAL_OUTPUT" | wc -l)

if [ "$PHYSICAL_COUNT" -gt 0 ]; then
    echo "$(date) - Physical monitor detected, disabling virtual display if active."
    # 禁用虚拟屏幕（如果存在）
    xrandr --output $VIRTUAL_OUTPUT --off 2>/dev/null
    exit 0
fi

echo "$(date) - No physical monitor detected, starting virtual display."

# -------------------------------
# 创建虚拟屏幕模式（1280x720 @ 60Hz）
# -------------------------------
# cvt 1280 720 60
# Modeline "1280x720_60.00" 74.50 1280 1344 1472 1664 720 723 728 748 -hsync +vsync

xrandr --newmode "$VIRTUAL_MODE" 74.50 1280 1344 1472 1664 720 723 728 748 -hsync +vsync 2>/dev/null
xrandr --addmode $VIRTUAL_OUTPUT $VIRTUAL_MODE 2>/dev/null
xrandr --output $VIRTUAL_OUTPUT --mode $VIRTUAL_MODE 2>/dev/null

echo "$(date) - Virtual display $VIRTUAL_MODE started."
