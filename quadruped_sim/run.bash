#!bin/bash -f

while $(sleep 60); do
  if systemctl is-system-running | grep -qE "degraded|running"; then
    break
  fi
done

source /opt/ros/noetic/setup.bash
source /home/jetson/quadruped_sim/devel/setup.bash
roscore &
roslaunch --wait quadruped run_robot_all.launch
