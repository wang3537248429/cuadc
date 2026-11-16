#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"
errors=0

fail() {
  printf '  [缺失] %s\n' "$1" >&2
  errors=$((errors + 1))
}

pass() {
  printf '  [正常] %s\n' "$1"
}

printf '[CUADC] 检查系统环境\n'

if [[ ! -r /etc/os-release ]]; then
  fail "无法读取 /etc/os-release"
else
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    pass "Ubuntu 22.04"
  else
    fail "只验证过 Ubuntu 22.04，当前为 ${PRETTY_NAME:-未知系统}"
  fi
fi

if [[ "$(dpkg --print-architecture 2>/dev/null || true)" == "amd64" ]]; then
  pass "amd64 架构"
else
  fail "当前发布包只验证过 amd64"
fi

for command in git cmake g++ python3 gzip sha256sum rg gz; do
  if command -v "$command" >/dev/null 2>&1; then
    pass "命令 $command"
  else
    fail "命令 $command"
  fi
done

if [[ -r /opt/ros/humble/setup.bash ]]; then
  pass "ROS 2 Humble"
else
  fail "/opt/ros/humble/setup.bash"
fi

bridge="/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
if [[ -x "$bridge" ]]; then
  pass "ROS/Gazebo Garden 图像与事件桥"
else
  fail "$bridge"
fi

gazebo_version="$(gz sim --versions 2>/dev/null | head -n 1 || true)"
if [[ "$gazebo_version" == 7.9.* ]]; then
  pass "Gazebo Garden ${gazebo_version}"
else
  fail "需要 Gazebo Garden 7.9.x，当前为 ${gazebo_version:-未安装}"
fi

if [[ "$errors" -ne 0 ]]; then
  printf '\n[CUADC] 环境检查失败，共 %d 项。请先运行：\n' "$errors" >&2
  printf '  sudo %s/install_system_deps.sh\n' "$script_dir" >&2
  exit 1
fi

printf '[CUADC] 系统环境检查通过。\n'
