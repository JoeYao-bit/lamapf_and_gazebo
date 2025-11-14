#!/bin/bash

# 检测物理屏幕
CONNECTED=$(xrandr | grep " connected" | wc -l)

if [ "$CONNECTED" -gt 0 ]; then
    echo "Physical monitor detected, skipping virtual display."
    exit 0
fi

echo "No monitor detected, starting virtual display."

# 创建虚拟屏幕
xrandr --setprovideroutputsource 1 0 2>/dev/null
xrandr --newmode "1280x720_60.00"  172.80  1920 2040 2248 2576  1080 1083 1088 1120 -hsync +vsync
xrandr --addmode VIRTUAL1 1280x720_60.00
xrandr --output VIRTUAL1 --mode 1280x720_60.00
