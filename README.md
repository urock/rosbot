ROSBot control mode development 

Инструкция по работе

# 1. Install Prerequisites
### 1. [Ubuntu 18.04](https://ubuntu.com) 
### 2. [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
### 3. Clone repository 
#### 3.1 создать директорию под catkin_ws 
#### 3.2 открыть терминал ( ctrl + alt + t )
*catkin_ws_path - путь до catkin_ws
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
*Перезапустить терминал*
```

# 2. Запуск симулятора
открыть терминал
```bash
cd catkin_ws_path
source devel/setup.zsh
roslaunch rosbot_controller run_simulation.launch traj_type:={sin, polygon}
```
## 2.1 Пример
```bash
cd catkin_ws_path
source devel/setup.zsh
roslaunch rosbot_controller run_simulation.launch traj_type:=-2.5sin1.0
```

# 3. Сбор данных
открыть терминал
```bash
cd catkin_ws_path
cd src/rosbot/rosbot_controller/src/ 
./test.zsh -t="5.0sin0.1 -5.0sin0.2 3.0sin3.0 2.0sin1.5" -v="1.5 2.5 3.5 4.5" -w="0.5 1.0 2.5 3.0"
```
* -t - список траекторий
* -v - список линейных скоростей
* -w - список угловых скоростей
Данные будут сохранены в *logger/output_data/*
