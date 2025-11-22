#!/bin/bash
USER_HOME="/home/user"
DESKTOP_DIR="$USER_HOME/Desktop"
TARGET_SCRIPT="/home/user/edit_rcS.bash"
DESKTOP_FILE="$DESKTOP_DIR/edit_rcS.desktop"

mkdir -p "$DESKTOP_DIR"

cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Remote QgroundControl
Comment=Connect QgroundControl from Host to Docker
Exec=$TARGET_SCRIPT
Icon=/home/user/images/qgroundcontrol_remote.png
Terminal=true
StartupNotify=false
Categories=Utility;
EOF

chmod +x "$DESKTOP_FILE"