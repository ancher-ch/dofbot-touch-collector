#!/bin/sh
set -eu
COLLECTOR_KIT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
exec /bin/sh "$COLLECTOR_KIT/launch-robot.sh"
