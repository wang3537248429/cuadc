#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
remove_dependencies=0

case "${1:-}" in
  '') ;;
  --dependencies) remove_dependencies=1 ;;
  -h|--help)
    printf '用法：%s [--dependencies]\n' "$0"
    printf '默认删除发布包、运行结果、缓存和构建产物。\n'
    printf '增加 --dependencies 后还会删除本地虚拟环境和已下载依赖。\n'
    exit 0
    ;;
  *)
    printf '未知参数：%s\n' "$1" >&2
    exit 2
    ;;
esac

generated_paths=(
  "$project_dir/dist"
  "$project_dir/gazebo_sim/.runtime"
  "$project_dir/gazebo_sim/runs"
  "$project_dir/gazebo_sim/.pytest_cache"
  "$project_dir/gazebo_sim/build"
  "$project_dir/gazebo_sim/src/cuadc_gazebo_sim.egg-info"
  "$project_dir/onboard/.runtime"
  "$project_dir/onboard/runs"
  "$project_dir/onboard/.pytest_cache"
  "$project_dir/onboard/build"
  "$project_dir/onboard/src/cuadc_onboard.egg-info"
  "$project_dir/ground_station/.pytest_cache"
  "$project_dir/ground_station/build"
)

if [[ "$remove_dependencies" -eq 1 ]]; then
  generated_paths+=(
    "$project_dir/gazebo_sim/.deps"
    "$project_dir/gazebo_sim/.venv"
    "$project_dir/onboard/.venv-onboard"
    "$project_dir/ground_station/.venv-windows"
  )
fi

for generated_path in "${generated_paths[@]}"; do
  if [[ -e "$generated_path" ]]; then
    rm -rf -- "$generated_path"
    printf '[已删除] %s\n' "${generated_path#"$project_dir/"}"
  fi
done

find "$project_dir" \
  -path "$project_dir/.git" -prune -o \
  -path "$project_dir/gazebo_sim/.venv" -prune -o \
  -path "$project_dir/onboard/.venv-onboard" -prune -o \
  -path "$project_dir/ground_station/.venv-windows" -prune -o \
  -path "$project_dir/gazebo_sim/.deps" -prune -o \
  -type d -name __pycache__ -prune -exec rm -rf -- {} +
find "$project_dir" \
  -path "$project_dir/.git" -prune -o \
  -path "$project_dir/gazebo_sim/.venv" -prune -o \
  -path "$project_dir/onboard/.venv-onboard" -prune -o \
  -path "$project_dir/ground_station/.venv-windows" -prune -o \
  -path "$project_dir/gazebo_sim/.deps" -prune -o \
  -type f \( -name '*.pyc' -o -name '*.pyo' \) -exec rm -f -- {} +

printf '[CUADC] 可重建文件已清理。\n'
