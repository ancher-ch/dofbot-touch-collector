#!/bin/sh
# Start the existing app; startup does not request position or torque writes.
set -eu
COLLECTOR_KIT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
COLLECTOR_PYTHON=${COLLECTOR_PYTHON:-/usr/bin/python3}
COLLECTOR_DATA_ROOT=${COLLECTOR_DATA_ROOT:-"$HOME/dofbot-data/touch_collector"}
COLLECTOR_LOG_DIR=${COLLECTOR_LOG_DIR:-"$HOME/dofbot-data/reports"}
if [ "$(uname -s)" != Linux ]; then
    printf '%s\n' 'Real hardware mode requires the compatible robot Linux system.' >&2
    exit 2
fi
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-"$HOME/.Xauthority"}
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
mkdir -p "$COLLECTOR_DATA_ROOT" "$COLLECTOR_LOG_DIR"
cd "$COLLECTOR_KIT/software"
exec "$COLLECTOR_PYTHON" -m touch_collector.app --fullscreen --root "$COLLECTOR_DATA_ROOT" >> "$COLLECTOR_LOG_DIR/touch_collector.log" 2>&1
