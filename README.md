# Gazebo simulation

This is a Gazebo simulation for a 4-wheeled robot that follows a white ball kept on a simulated house floor.

Move the ball, and as long as the robot's camera can see the white ball, it will follow along.

## Setup Instructions:

### git clone https://github.com/sumit280188/Gazebo-Simulation.git



```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch

Open another terminal:
cd ~/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch

Open yet another terminal:
cd ~/catkin_ws/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view
