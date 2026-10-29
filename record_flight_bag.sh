#!/bin/bash
# 轻量录制：bash record_flight_bag.sh [保存目录]；Ctrl+C 正常结束。
set -e

BAG_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${BAG_WORKSPACE}/devel/setup.bash"
BAG_DIR="${1:-${BAG_WORKSPACE}/Bag}"
mkdir -p "${BAG_DIR}"
BAG_PREFIX="${BAG_DIR}/flight_$(date +%Y%m%d_%H%M%S)_$$"

# 不记录图像和点云；保留飞行、检测、规划及任务日志，按约64MB分卷。
TOPICS=(
    /rflysim/uav1/local/odom
    /mavros/local_position/odom
    /mavros/state
    /mavros/setpoint_raw/local
    /objects
    /planning/pos_cmd
    /planning/bspline
    /move_base_simple/goal
    /mission_command
    /mission_result
    /mission_abort
    /aruco/detected_id
    /rosout_agg
)

echo "[Bag] Recording to ${BAG_PREFIX}_*.bag (LZ4, 64MB splits)"
exec rosbag record --lz4 --split --size=64 --buffsize=64 \
    -O "${BAG_PREFIX}" "${TOPICS[@]}" __name:=flight_bag_recorder
