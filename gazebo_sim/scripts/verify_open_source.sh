#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -x "$project_dir/../scripts/verify_release.sh" ]]; then
  exec "$project_dir/../scripts/verify_release.sh"
fi
cd "$project_dir"
for script in scripts/*.sh; do bash -n "$script"; done
sha256sum --check OFFICIAL_ASSET_SHA256SUMS
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
if [[ "${CUADC_SKIP_SDF:-0}" != "1" ]]; then gz sdf -k models/cuadc_twin/model.sdf >/dev/null; fi
