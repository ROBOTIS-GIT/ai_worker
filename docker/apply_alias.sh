# This script applies aliases inside the ai_worker container for developer convenience.
{
  echo ''
  echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp'
  echo "export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'"
  echo "# alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'"
  echo ''
  echo "alias ws='cd ~/ros2_ws && cb && source install/setup.bash'"
  echo ''
  echo "alias ffw_mini_leader='ros2 launch ffw_bringup ffw_lg2_mini_leader_ai.launch.py'"
  echo "alias ffw_a3_leader='ros2 launch ffw_bringup ffw_a3_ai.launch.py'"
} >> ~/.bashrc
