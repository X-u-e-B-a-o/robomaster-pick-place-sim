#!/usr/bin/env bash
set -e

BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 防止误加载旧工作区。
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH

source /opt/ros/humble/setup.bash

if [ ! -f "$BUNDLE_ROOT/install/setup.bash" ]; then
    echo "首次运行：正在编译本压缩包内的三个 ROS 2 包……"

    cd "$BUNDLE_ROOT"

    colcon --log-base "$BUNDLE_ROOT/log" build \
        --base-paths "$BUNDLE_ROOT/src" \
        --build-base "$BUNDLE_ROOT/build" \
        --install-base "$BUNDLE_ROOT/install" \
        --symlink-install \
        --packages-select \
        ros_gz_sim \
        gz_ros2_control \
        robomaster_pick_place_sim
fi

source "$BUNDLE_ROOT/install/setup.bash"

PKG_PREFIX="$(ros2 pkg prefix robomaster_pick_place_sim)"

export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/home/nvidia/.Xauthority}"
export GZ_VERSION=fortress

export IGN_GAZEBO_RESOURCE_PATH="$PKG_PREFIX/share:$BUNDLE_ROOT/src"
export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"

echo "加载包：$PKG_PREFIX"
echo "显示器：$DISPLAY"
echo "Gazebo 将保持暂停，请暂时不要点击 Play。"

exec ros2 launch \
    robomaster_pick_place_sim \
    static_model.launch.py
