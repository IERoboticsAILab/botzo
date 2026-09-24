#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
# The repo is bind-mounted at /ws/src/botzo; build output lives in /ws/{build,install,log}.
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
