#!/bin/sh
# Run from the robot's graphical Linux desktop. Never selects real hardware.
set -eu
COLLECTOR_KIT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
COLLECTOR_PYTHON=${COLLECTOR_PYTHON:-/usr/bin/python3}
cd "$COLLECTOR_KIT/software"
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
exec "$COLLECTOR_PYTHON" -m touch_collector.app --demo --root "$COLLECTOR_KIT/demo-data"
