#!/bin/bash
# stack.sh start|stop|restart: the robot controllers from main in the restore_main container.
#   start: controller_manager.launch.py in robot mode, hand on, safety filter on, scene boxes allowed.
stop() {
  docker exec restore_main bash -c '
    pkill -INT -f "ros2 launch lara5_controller_manager" || true
    for i in $(seq 1 40); do pgrep -f "ros2_control_node|safety_filter_node" >/dev/null || break; sleep 0.5; done
    pkill -9 -f "ros2_control_node|safety_filter_node|robot_state_publisher" 2>/dev/null; true'
  echo stopped
}
start() {
  ping -c 1 -W 2 192.168.2.13 >/dev/null || { echo "arm 192.168.2.13 not reachable"; exit 1; }
  ping -c 1 -W 2 192.168.2.75 >/dev/null || { echo "hand 192.168.2.75 not reachable"; exit 1; }
  rm -f /home/aras/restore_run/controllers.log
  docker exec -d restore_main bash -c 'source /ws/install/setup.bash; exec ros2 launch lara5_controller_manager \
    controller_manager.launch.py deployment_mode:=robot hands:=on safety:=on allow_scene_updates:=true rviz:=false \
    > /run_data/controllers.log 2>&1'
  for i in $(seq 1 60); do
    grep -q "Robot model built successfully" /home/aras/restore_run/controllers.log 2>/dev/null && \
    grep -q "Configured and activated.*arm_trajectory_controller" /home/aras/restore_run/controllers.log && break
    sleep 1
  done
  grep -E "Connected to hand|Configured and activated|Safety filter ready|ERROR" /home/aras/restore_run/controllers.log | cut -c1-160
}
case "$1" in
  start) start ;;
  stop) stop ;;
  restart) stop; sleep 2; start ;;
  *) echo "usage: stack.sh start|stop|restart"; exit 2 ;;
esac
