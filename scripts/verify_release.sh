#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
python_bin="${CUADC_PYTHON:-python3}"

printf '[CUADC] 三模块开源检查\n'
"$python_bin" scripts/privacy_audit.py --json dist/validation/privacy.json
for directory in scripts gazebo_sim/scripts onboard/scripts ground_station; do
  for script in "$directory"/*.sh; do
    [[ -f "$script" ]] && bash -n "$script"
  done
done
"$python_bin" scripts/check_local_resources.py
(cd gazebo_sim && sha256sum --check OFFICIAL_ASSET_SHA256SUMS)
pytest_arguments=()
if [[ -n "${CUADC_TEST_TMPDIR:-}" ]]; then
  pytest_arguments+=(--basetemp "$CUADC_TEST_TMPDIR")
fi
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
  "$python_bin" -m pytest gazebo_sim/tests onboard/tests ground_station/tests scripts/tests \
    --import-mode=importlib -q -p no:cacheprovider --junitxml=dist/validation/tests.xml "${pytest_arguments[@]}"
if [[ "${CUADC_SKIP_SDF:-0}" != "1" ]]; then
  gz sdf -k gazebo_sim/models/cuadc_twin/model.sdf >/dev/null
  gz sdf -k gazebo_sim/models/cuadc_world_assets/model.sdf >/dev/null
fi
printf '[CUADC] 三模块开源检查通过\n'
