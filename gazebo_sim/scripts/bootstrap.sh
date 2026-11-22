#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法：bootstrap.sh [--no-vision]

默认安装完整视觉仿真，并要求靶标图片已经准备好；运行权重归档随源码提供。
--no-vision 只安装基础仿真依赖；适用于没有视觉资源的干净源码检出，
            与 run_sim.sh --no-vision 配合使用。
EOF
}

VISION_REQUIRED=1
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --no-vision)
      VISION_REQUIRED=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[CUADC] 未知参数：%s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"

python_major_minor="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "$python_major_minor" != "3.10" ]]; then
  printf '[CUADC] 仿真环境需要 Python 3.10，当前为 %s。\n' "$python_major_minor" >&2
  exit 1
fi
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
if [[ "$VISION_REQUIRED" == "1" ]]; then
  if ! (
    cd "$project_dir"
    sha256sum --check models/vision/MODEL_SHA256SUMS >/dev/null 2>&1
  ); then
    missing_archives=()
    for archive in task1_v4m_best.pt.gz task2_sim_digits_v1.pt.gz; do
      [[ -f "${project_dir}/models/vision/$archive" ]] || missing_archives+=("$archive")
    done
    if [[ "${#missing_archives[@]}" -gt 0 ]]; then
      printf '\n[CUADC] 缺少视觉模型归档；请确认公开权重文件没有被清理。\n' >&2
      printf '[CUADC] 缺少模型归档：%s\n' "${missing_archives[*]}" >&2
      printf '[CUADC] 请准备自有授权资源，或使用 bootstrap.sh --no-vision。\n' >&2
      printf '[CUADC] 资源说明：docs/资源与许可.md\n\n' >&2
      exit 1
    fi
    gzip -d -k -f "${project_dir}/models/vision/task1_v4m_best.pt.gz"
    gzip -d -k -f "${project_dir}/models/vision/task2_sim_digits_v1.pt.gz"
  fi
  (
    cd "$project_dir"
    sha256sum --check models/vision/MODEL_SHA256SUMS
  )
  if [[ -f "$project_dir/../scripts/check_local_resources.py" ]]; then
    python3 "$project_dir/../scripts/check_local_resources.py" \
      --project gazebo_sim --require
  fi
else
  printf '[CUADC] --no-vision：跳过本地模型校验和视觉依赖。\n'
fi

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
  if [[ "$VISION_REQUIRED" == "1" ]]; then
    "${project_dir}/.venv/bin/python" -m pip install \
      --requirement "$CUADC_SIM_REQUIREMENTS"
  else
    # Avoid installing torch/Ultralytics when the caller explicitly selected
    # the no-vision mode.  Keep this list in a normal requirements file so it
    # can be inspected and reproduced independently.
    "${project_dir}/.venv/bin/python" -m pip install \
      --requirement "$project_dir/requirements-core.txt" \
      -e .
  fi
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
