#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"
deps_dir="${project_dir}/.deps"
ardupilot_dir="${deps_dir}/ardupilot"
gazebo_plugin_dir="${deps_dir}/ardupilot_gazebo"
runtime_deps_dir="${deps_dir}/runtime"
ardupilot_runtime_dir="${runtime_deps_dir}/ardupilot-4.6.3"
gazebo_runtime_dir="${runtime_deps_dir}/ardupilot-gazebo-garden"
# Immutable official Plane-4.6.3 tag commit. Do not use the moving
# ArduPlane-stable branch here.
ardupilot_commit="3fc7011a7d3dc047cbb17d8bd98ee94577d144c6"
gazebo_plugin_commit="082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"

mkdir -p "$deps_dir" "$runtime_deps_dir" "${project_dir}/.runtime"
"$script_dir/check_environment.sh"

# The selected official SITL_Models assets are vendored and pinned in
# OFFICIAL_ASSETS.md. Keeping a second source checkout consumed
# over a gigabyte without being used at build or runtime, so bootstrap verifies
# the vendored files directly instead.
(
  cd "$CUADC_GAZEBO_DIR"
  sha256sum --check OFFICIAL_ASSET_SHA256SUMS
)

# Local weights require separate provenance; no automatic model downloads.
if ! (
  cd "$project_dir"
  sha256sum --check models/vision/MODEL_SHA256SUMS >/dev/null 2>&1
); then
  if [[ -f "${project_dir}/models/vision/task1_v4m_best.pt.gz" ]]; then
    gzip -d -k -f "${project_dir}/models/vision/task1_v4m_best.pt.gz"
    gzip -d -k -f "${project_dir}/models/vision/task2_sim_digits_v1.pt.gz"
  else
    printf '\n[CUADC] 源码不分发授权未确认的模型和靶标；请先配置自有资源。\n' >&2
    printf '[CUADC] 操作说明见 docs/资源与许可.md 和 gazebo_sim/models/vision/README.md。\n\n' >&2
    exit 1
  fi
fi
(
  cd "$project_dir"
  sha256sum --check models/vision/MODEL_SHA256SUMS
)

ardupilot_ready=0
if [[ -x "${ardupilot_runtime_dir}/arduplane" ]] \
  && [[ -f "${ardupilot_runtime_dir}/COMMIT" ]] \
  && grep -qx "$ardupilot_commit" "${ardupilot_runtime_dir}/COMMIT"; then
  ardupilot_ready=1
fi

if [[ "$ardupilot_ready" != "1" ]]; then
  if [[ ! -d "${ardupilot_dir}/.git" ]]; then
    git clone --filter=blob:none --no-checkout \
      https://github.com/ArduPilot/ardupilot.git "$ardupilot_dir"
  fi
  git -C "$ardupilot_dir" fetch --depth 1 origin "$ardupilot_commit"
  git -C "$ardupilot_dir" checkout --detach "$ardupilot_commit"
  git -C "$ardupilot_dir" submodule update --init --depth 1 \
    modules/mavlink modules/waf modules/DroneCAN/DSDL \
    modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard
  git -C "$ardupilot_dir" submodule update --init --force --depth 1 modules/waf
  test -f "${ardupilot_dir}/modules/waf/waf-light"
  (
    cd "$ardupilot_dir"
    ./waf configure --board sitl
    ./waf plane -j"$(nproc)"
  )
  mkdir -p "$ardupilot_runtime_dir"
  install -m 0755 "${ardupilot_dir}/build/sitl/bin/arduplane" \
    "${ardupilot_runtime_dir}/arduplane"
  install -m 0755 "${ardupilot_dir}/Tools/scripts/dumpstack.sh" \
    "${ardupilot_runtime_dir}/dumpstack.sh"
  install -m 0755 "${ardupilot_dir}/Tools/scripts/dumpcore.sh" \
    "${ardupilot_runtime_dir}/dumpcore.sh"
  install -m 0644 "${ardupilot_dir}/COPYING.txt" \
    "${ardupilot_runtime_dir}/COPYING.txt"
  printf '%s\n' "$ardupilot_commit" >"${ardupilot_runtime_dir}/COMMIT"
fi

gazebo_plugin_ready=0
if [[ -f "${gazebo_runtime_dir}/libArduPilotPlugin.so" ]] \
  && [[ -f "${gazebo_runtime_dir}/COMMIT" ]] \
  && grep -qx "$gazebo_plugin_commit" "${gazebo_runtime_dir}/COMMIT"; then
  gazebo_plugin_ready=1
fi

if [[ "$gazebo_plugin_ready" != "1" ]]; then
  if [[ ! -d "${gazebo_plugin_dir}/.git" ]]; then
    git clone --filter=blob:none \
      https://github.com/ArduPilot/ardupilot_gazebo.git "$gazebo_plugin_dir"
  fi
  git -C "$gazebo_plugin_dir" fetch --depth 1 origin "$gazebo_plugin_commit"
  git -C "$gazebo_plugin_dir" checkout --detach "$gazebo_plugin_commit"

  airspeed_patch="$CUADC_GAZEBO_DIR/patches/ardupilot_gazebo_airspeed.patch"
  if git -C "$gazebo_plugin_dir" apply --reverse --check "$airspeed_patch" 2>/dev/null; then
    printf 'ArduPilot Gazebo airspeed patch already applied.\n'
  elif git -C "$gazebo_plugin_dir" apply --check "$airspeed_patch"; then
    git -C "$gazebo_plugin_dir" apply "$airspeed_patch"
  else
    printf 'Cannot apply %s to plugin commit %s\n' \
      "$airspeed_patch" "$gazebo_plugin_commit" >&2
    exit 1
  fi
  finite_state_patch="$CUADC_GAZEBO_DIR/patches/ardupilot_gazebo_finite_state.patch"
  if git -C "$gazebo_plugin_dir" apply --reverse --check "$finite_state_patch" 2>/dev/null; then
    printf 'ArduPilot Gazebo finite-state guard patch already applied.\n'
  elif git -C "$gazebo_plugin_dir" apply --check "$finite_state_patch"; then
    git -C "$gazebo_plugin_dir" apply "$finite_state_patch"
  else
    printf 'Cannot apply %s after the airspeed patch on plugin commit %s\n' \
      "$finite_state_patch" "$gazebo_plugin_commit" >&2
    exit 1
  fi

  export GZ_VERSION=garden
  cmake -S "$gazebo_plugin_dir" -B "${gazebo_plugin_dir}/build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build "${gazebo_plugin_dir}/build" -j"$(nproc)"
  mkdir -p "$gazebo_runtime_dir"
  install -m 0755 "${gazebo_plugin_dir}/build/libArduPilotPlugin.so" \
    "${gazebo_runtime_dir}/libArduPilotPlugin.so"
  install -m 0644 "${gazebo_plugin_dir}/LICENSE.md" \
    "${gazebo_runtime_dir}/LICENSE.md"
  printf '%s\n' "$gazebo_plugin_commit" >"${gazebo_runtime_dir}/COMMIT"
fi

python3 -m venv "${project_dir}/.venv"
"${project_dir}/.venv/bin/python" -m pip install \
  --upgrade "pip==26.1.2" "setuptools==80.9.0" "wheel==0.46.3"
(
  cd "$project_dir"
  "${project_dir}/.venv/bin/python" -m pip install \
    --requirement "$CUADC_SIM_REQUIREMENTS"
)

cmake -S "$CUADC_GAZEBO_DIR/plugins" \
  -B "${project_dir}/.runtime/plugins-build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "${project_dir}/.runtime/plugins-build" -j"$(nproc)"

test -x "${ardupilot_runtime_dir}/arduplane"
test -f "${gazebo_runtime_dir}/libArduPilotPlugin.so"
test -x "${project_dir}/.venv/bin/mavproxy.py"

# Source and build trees are only staging areas.  The fixed-version runtime
# package above is all the simulator uses; removing staging avoids retaining
# roughly one gigabyte of unrelated source, Git objects and build products.
for staging_dir in "$ardupilot_dir" "$gazebo_plugin_dir"; do
  if [[ -d "$staging_dir" ]]; then
    find "$staging_dir" -xdev -depth -delete
  fi
done

printf 'Bootstrap complete. Run: %s/run_sim.sh\n' "$script_dir"
