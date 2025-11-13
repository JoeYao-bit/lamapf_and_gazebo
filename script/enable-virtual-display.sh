#!/bin/bash
# 启用虚拟显示输出（无屏幕也能启动图形桌面）
# 适用于 Ubuntu 20.04 / 22.04 / 24.04

set -e

echo "🧩 Installing dummy video driver..."
sudo apt update
sudo apt install -y xserver-xorg-video-dummy

echo "🧩 Creating dummy display config..."
sudo mkdir -p /usr/share/X11/xorg.conf.d/
sudo tee /usr/share/X11/xorg.conf.d/20-dummy.conf > /dev/null <<'EOF'
Section "Device"
  Identifier "DummyDevice"
  Driver "dummy"
  VideoRam 256000
EndSection

Section "Monitor"
  Identifier "DummyMonitor"
  HorizSync 28.0-80.0
  VertRefresh 48.0-75.0
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

Section "ServerLayout"
  Identifier "DummyLayout"
  Screen "DummyScreen"
EndSection
EOF

echo "✅ Dummy display configuration applied."

# 检查显示管理器
if systemctl list-unit-files | grep -q gdm.service; then
  DISPLAY_MANAGER=gdm
elif systemctl list-unit-files | grep -q lightdm.service; then
  DISPLAY_MANAGER=lightdm
elif systemctl list-unit-files | grep -q sddm.service; then
  DISPLAY_MANAGER=sddm
else
  DISPLAY_MANAGER=""
fi

if [ -n "$DISPLAY_MANAGER" ]; then
  echo "🔄 Restarting display manager ($DISPLAY_MANAGER)..."
  sudo systemctl restart $DISPLAY_MANAGER
else
  echo "⚠️ Could not detect display manager automatically. Please reboot manually."
fi

echo "🎉 Virtual display enabled. You can now access GUI remotely without a monitor."
