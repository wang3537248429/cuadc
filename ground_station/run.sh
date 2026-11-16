#!/usr/bin/env bash
set -euo pipefail

ground_station_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$ground_station_dir${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m cuadc_ground_station "$@"
