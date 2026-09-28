gregory@gregory-NMH-WDX9:~$ ros2 topic list | grep -i vehicle_qr
ros2 topic list | grep -i pose | grep -i qr
/vehicle_qr2/cmd_vel
/vehicle_qr3/cmd_vel
/vehicle_qr4/cmd_vel
/vehicle_qr5/cmd_vel
/vehicle_qr6/cmd_vel
/vehicle_qr7/cmd_vel
/vehicle_qr8/cmd_vel
/vehicle_qr9/cmd_vel
gregory@gregory-NMH-WDX9:~$ ros2 topic list | grep -i vehicle_qr
/vehicle_qr2/cmd_vel
/vehicle_qr3/cmd_vel
/vehicle_qr4/cmd_vel
/vehicle_qr5/cmd_vel
/vehicle_qr6/cmd_vel
/vehicle_qr7/cmd_vel
/vehicle_qr8/cmd_vel
/vehicle_qr9/cmd_vel
gregory@gregory-NMH-WDX9:~$ ros2 topic list | grep -i pose | grep -i qr
gregory@gregory-NMH-WDX9:~$ gz topic -l | grep vehicle_qr
/model/vehicle_qr2/cmd_vel
/model/vehicle_qr3/cmd_vel
/model/vehicle_qr3/odometry
/model/vehicle_qr3/tf
/model/vehicle_qr4/cmd_vel
/model/vehicle_qr5/cmd_vel
/model/vehicle_qr6/cmd_vel
/model/vehicle_qr7/cmd_vel
/model/vehicle_qr7/odometry
/model/vehicle_qr7/tf
/model/vehicle_qr8/cmd_vel
/model/vehicle_qr9/cmd_vel
/model/vehicle_qr9/odometry
/model/vehicle_qr9/tf