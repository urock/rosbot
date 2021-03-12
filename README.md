ROSBot control mode development 

Инструкция по работе

# 1. Install Prerequisites
### 1. [Install ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu) 
### 2. [install ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
### 3. Clone repository 
#### 3.1 create catkin_ws 
#### 3.2 open terminal ctrl + alt + t
```bash
cd catkin_ws_path
mkdir -p catkin_ws/src
cd catkin_ws
wstool init ./src
cd src
git clone git@github.com:urock/rosbot.git 
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y 
git submodule update --init --recursive 
catkin build
```
# 2. Run simulation
open terminal
```bash
cd catkin_ws_path
source devel/setup.zsh
roslaunch rosbot2 urock_system.launch
```
open another terminal
```bash
roslaunch rosbot_controller folow_path.launch traj_type:={sin,polygon}
```
    roslaunch rosbot_controller folow_path.launch traj_type:={sin,polygon}
