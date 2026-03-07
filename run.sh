#!/bin/bash
# Launch PICO VR Teleop for Lebai LM3
# 启动 PICO VR 遥操作乐白LM3
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$SCRIPT_DIR:${LD_LIBRARY_PATH:-}"
cd "$SCRIPT_DIR"
exec python3 main.py "$@"
