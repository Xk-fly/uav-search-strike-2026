#!/bin/bash
# 轻量录制：bash record_flight_bag.sh [保存目录]；Ctrl+C 正常结束。
set -e

BAG_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${BAG_WORKSPACE}/devel/setup.bash"
BAG_DIR="${1:-${BAG_WORKSPACE}/Bag}"
mkdir -p "${BAG_DIR}"
BAG_PREFIX="${BAG_DIR}/flight_$(date +%Y%m%d_%H%M%S)_$$"

# 稀疏点云每秒最多2000点；版本及右墙净空便于回放，不录原始图像/全量点云。
python3 "${BAG_WORKSPACE}/flight_bag_context.py" &
CONTEXT_PID=$!

stop_recording() {
    trap '' INT TERM
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
}
trap stop_recording INT TERM
trap 'kill -TERM "${CONTEXT_PID}" 2>/dev/null || true; wait "${CONTEXT_PID}" 2>/dev/null || true' EXIT

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
    /visual_servo/right_wall_clearance
    /flight_bag/version
    /flight_bag/cloud_sparse
)

echo "[Bag] Recording to ${BAG_PREFIX}_*.bag (LZ4, 64MB splits)"
rosbag record --lz4 --split --size=64 --buffsize=64 \
    -O "${BAG_PREFIX}" "${TOPICS[@]}" __name:=flight_bag_recorder &
RECORDER_PID=$!
wait "${RECORDER_PID}"
