#!/bin/bash

set -euo pipefail

# Ensure we run from the project directory
cd /home/pi/botzo/main

# Export headless SDL to avoid display issues
export SDL_VIDEODRIVER=dummy
export SDL_AUDIODRIVER=dummy

# Activate python (system python3 assumed); extend PATH if needed
PYTHON_BIN=python3

# Log directory
LOG_DIR=/home/pi/botzo/logs
mkdir -p "$LOG_DIR"

TS=$(date +"%Y%m%d-%H%M%S")
LOG_FILE="$LOG_DIR/botzo-$TS.log"

echo "[start_bot] Waiting for controller device to appear..."
# Optional max wait in seconds; 0 means wait indefinitely
MAX_WAIT_SEC=${MAX_WAIT_SEC:-0}
START_TIME=$(date +%s)
while true; do
  if ls /dev/input/js* >/dev/null 2>&1; then
    echo "[start_bot] Controller device file detected."
    # Additional wait for controller to be fully ready
    sleep 3
    # Verify pygame can actually see the controller
    if "$PYTHON_BIN" -c "import os; os.environ['SDL_VIDEODRIVER']='dummy'; os.environ['SDL_AUDIODRIVER']='dummy'; import pygame; pygame.init(); pygame.joystick.init(); count=pygame.joystick.get_count(); pygame.quit(); exit(0 if count > 0 else 1)" 2>/dev/null; then
      echo "[start_bot] Controller verified and ready. Proceeding."
      break
    else
      echo "[start_bot] Controller device exists but pygame can't detect it yet. Retrying..."
    fi
  fi
  NOW=$(date +%s)
  ELAPSED=$((NOW - START_TIME))
  if [ "$MAX_WAIT_SEC" -gt 0 ] && [ "$ELAPSED" -ge "$MAX_WAIT_SEC" ]; then
    echo "[start_bot] Timed out waiting for controller after ${ELAPSED}s. Exiting for systemd restart."
    exit 1
  fi
  echo "[start_bot] Controller not ready yet. Waiting..."
  sleep 2
done

echo "[start_bot] Launching IK_moving_parallel.py, logging to $LOG_FILE"
exec "$PYTHON_BIN" /home/pi/botzo/main/IK_moving_parallel.py >> "$LOG_FILE" 2>&1


