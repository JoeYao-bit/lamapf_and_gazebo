#!/bin/bash
# 自动启用虚拟显示：仅当未检测到物理屏幕时

if ! xrandr | grep -q " connected"; then
  echo "🖥️ No display detected, enabling virtual display..."
  sudo apt install -y xserver-xorg-video-dummy
  sudo tee /usr/share/X11/xorg.conf.d/20-dummy.conf > /dev/null <<'EOF'
Section "Device"
  Identifier "DummyDevice"
  Driver "dummy"
EndSection

Section "Monitor"
  Identifier "DummyMonitor"
  Modeline "1920x1080" 172.80 1920 2040 2248 2576 1080 1083 1088 1120
  Option "PreferredMode" "1920x1080"
EndSection

Section "Screen"
  Identifier "DummyScreen"
  Device "DummyDevice"
  Monitor "DummyMonitor"
  DefaultDepth 24
  SubSection "Display"
    Depth 24
    Modes "1920x1080"
  EndSubSection
EndSection
EOF
  sudo systemctl restart gdm
else
  echo "✅ Physical monitor detected, virtual display not needed."
fi
