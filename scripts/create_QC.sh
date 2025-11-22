#!/bin/bash
USER_HOME="/home/user"
DESKTOP_DIR="$USER_HOME/Desktop"
LAUNCHER_SCRIPT="/home/user/qgroundcontrol/usr/bin/QGroundControl"
DESKTOP_FILE="$DESKTOP_DIR/QGroundControl.desktop"

mkdir -p "$DESKTOP_DIR"

cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=QGroundControl
Comment=Ground Control Station for UAVs
Exec=$LAUNCHER_SCRIPT
Icon=/home/user/images/qgroundcontrol.png
Terminal=false
StartupNotify=false
Categories=Utility;Development;
EOF

chmod +x "$DESKTOP_FILE"