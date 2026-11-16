#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=layout.sh
source "$script_dir/layout.sh"
project_dir="$CUADC_PROJECT_DIR"

if [[ ! -r /etc/os-release ]]; then
  printf '[CUADC] 无法识别操作系统。\n' >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  printf '[CUADC] 自动安装仅支持 Ubuntu 22.04 amd64；当前为 %s。\n' \
    "${PRETTY_NAME:-未知系统}" >&2
  exit 1
fi
if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  printf '[CUADC] 自动安装仅验证过 amd64。\n' >&2
  exit 1
fi

if [[ "$EUID" -eq 0 ]]; then
  sudo_command=()
else
  if ! command -v sudo >/dev/null 2>&1; then
    printf '[CUADC] 需要 root 权限或 sudo。\n' >&2
    exit 1
  fi
  sudo_command=(sudo)
fi

temporary_dir="$(mktemp -d)"
cleanup() {
  find "$temporary_dir" -xdev -depth -delete 2>/dev/null || true
}
trap cleanup EXIT

printf '[CUADC] 安装基础工具并配置 ROS/Gazebo 官方软件源。\n'
"${sudo_command[@]}" apt-get update
"${sudo_command[@]}" apt-get install -y \
  build-essential ca-certificates cmake curl gdb git gnupg lsb-release \
  ninja-build pkg-config python3-dev python3-pip python3-venv ripgrep \
  software-properties-common
"${sudo_command[@]}" add-apt-repository -y universe

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  --output "${temporary_dir}/ros-archive-keyring.gpg"
install -m 0644 "${temporary_dir}/ros-archive-keyring.gpg" \
  "${temporary_dir}/ros-archive-keyring-installed.gpg"
"${sudo_command[@]}" install -m 0644 \
  "${temporary_dir}/ros-archive-keyring-installed.gpg" \
  /usr/share/keyrings/ros-archive-keyring.gpg
printf 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' \
  >"${temporary_dir}/ros2.list"
"${sudo_command[@]}" install -m 0644 "${temporary_dir}/ros2.list" \
  /etc/apt/sources.list.d/ros2.list

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  --output "${temporary_dir}/pkgs-osrf-archive-keyring.gpg"
"${sudo_command[@]}" install -m 0644 \
  "${temporary_dir}/pkgs-osrf-archive-keyring.gpg" \
  /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main\n' \
  >"${temporary_dir}/gazebo-stable.list"
"${sudo_command[@]}" install -m 0644 "${temporary_dir}/gazebo-stable.list" \
  /etc/apt/sources.list.d/gazebo-stable.list

"${sudo_command[@]}" apt-get update
"${sudo_command[@]}" apt-get install -y \
  gz-garden libgz-sim7-dev ros-humble-ros-base ros-humble-ros-gzgarden

"$script_dir/check_environment.sh"
printf '[CUADC] 系统依赖安装完成。下一步运行：%s/bootstrap.sh\n' "$script_dir"
