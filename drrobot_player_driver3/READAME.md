
First copy the source code to /src folder. 
terminal --1: roscore
terminal -- 2:
cd catkin_ws
catkin_make
cd src
cd drrobot_player_driver3
rosparam load drrobotplayer.yaml
cd ..
cd ..
source ./devel/setup.bash
rosrun drrobot_player_driver3 drrobot_player_driver3_node

(this will publish all the sensor data and receive motor command)

terminal -- 3:
cd catkin_ws
source ./devel/setup.bash
(check motor sensor)
rostopic echo /drrobot_motor

(check motor driver baord )
rostopic echo /drrobot_motorboard

(check imu)
rostopic echo /drrobot_imu
(check GPS)
rostopic echo /drrobot_gps

(send command to robot)
below command will release emergency for motor driver board
rostopic pub -1 /drrobot_motor_cmd std_msgs/String "MM0\!MG"
belwo command will drive the robot move forward(open loop/velocity control)
rostopic pub -1 /drrobot_motor_cmd std_msgs/String "MM0\!M 500 -500"
below command will stop the robot
rostopic pub -1 /drrobot_motor_cmd std_msgs/String "MM0\!M 0 0"		


