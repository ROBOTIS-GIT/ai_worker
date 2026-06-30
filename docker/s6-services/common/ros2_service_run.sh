#!/bin/bash
# Reusable ROS2 service run script template
# This script launches ROS2 commands for any service
# Usage:
#   SERVICE_NAME=<name> ROS2_COMMAND="<full command>" /path/to/ros2_service_run.sh
#   If ROS2_COMMAND is not set, defaults to: ros2 launch ffw_bringup ${SERVICE_NAME}.launch.py
# Note: This script is called via /command/with-contenv, so environment is already set up

set -e

# Service name must be provided via environment variable
SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

# Set ROS_DOMAIN_ID if not already set (default to 30)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export COLCON_WS=${COLCON_WS:-/root/ros2_ws}

# Enable s6-overlay debug logging (set S6_VERBOSITY=1 for more verbose output)
export S6_VERBOSITY=${S6_VERBOSITY:-1}

echo "[${SERVICE_NAME}] Starting service..."
echo "[${SERVICE_NAME}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[${SERVICE_NAME}] ROS_DISTRO=${ROS_DISTRO}"
echo "[${SERVICE_NAME}] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[${SERVICE_NAME}] COLCON_WS=${COLCON_WS}"
echo "[${SERVICE_NAME}] PID: $$"

# Record process group ID so the finish script can target the whole group
PGID=$(ps -o pgid= -p $$ | tr -d ' ')
echo "[${SERVICE_NAME}] Process group: ${PGID}"
echo "${PGID}" > /run/${SERVICE_NAME}.pgid || true

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash

# Determine the command to execute
# If ROS2_COMMAND is set, use it; otherwise, use the default launch command
if [ -n "${ROS2_COMMAND}" ]; then
    ROS2_CMD="${ROS2_COMMAND}"
    echo "[${SERVICE_NAME}] Executing custom command: ${ROS2_CMD}"
else
    ROS2_CMD="ros2 launch ffw_bringup ${SERVICE_NAME}.launch.py"
    echo "[${SERVICE_NAME}] Executing default command: ${ROS2_CMD}"
fi

# Execute the ROS2 command as a background job in its own process group and
# forward shutdown signals to the whole group. ROS 2 launch often needs a group
# SIGINT so all child nodes get a graceful shutdown request.
set -m
bash -c "${ROS2_CMD}" &
CHILD_PID=$!
echo "[${SERVICE_NAME}] Child PID/process group: ${CHILD_PID}"
echo "${CHILD_PID}" > /run/${SERVICE_NAME}.pgid || true

shutdown_child() {
    SIGNAL="${1:-INT}"
    echo "[${SERVICE_NAME}] Forwarding SIG${SIGNAL} to process group ${CHILD_PID}"
    kill -"${SIGNAL}" -"${CHILD_PID}" 2>/dev/null || true
}

trap 'shutdown_child INT' INT
trap 'shutdown_child TERM' TERM
trap 'shutdown_child HUP' HUP

wait "${CHILD_PID}"
