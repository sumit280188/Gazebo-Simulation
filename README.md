This is a Gazebo-simulation for a 4-wheeled robot that follows a white ball kept on a simulated house floor, move the ball and, as long as the robot's camera can see the white ball, it will follow along.

open a terminal:
catkin_make your package

##cd ~/catkin_ws/
##source devel/setup.bash
##roslaunch my_robot world.launch

In another terminal:
cd ~/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch

To see the camera's raw images:
In another terminal:
cd ~/catkin_ws/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view
