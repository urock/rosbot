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

# 4. Описание launch файлов

## 4.1 run_simulation.launch
* Расположение rosbot_controller/launch/run_simulation.launch
* Что делает: Запускает gazebo, спавнит rosbot, публикует траекторию, запускает контроллер для следования rosobot по пути
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
	3. traj_type - тип траектории {sin polygon from_file}
	4. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller run_simulation.launch traj_type:=2.5sin0.2 rviz:=true gui:=false 
```

## 4.2 spawn_rosbot.launch
* Расположение rosbot2/launch/spawn_rosbot.launch
* Что делает: Запускает gazebo, спавгит rosbot
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
* Пример использования
```
roslaunch rosbot2 spawn_rosbot.launch rviz:=true gui:=false 
```

## 4.3 publish_path.launch
* Расположение rosbot_controller/launch/publish_path.launch
* Что делает: публикует траекторию для rosbot
* Аргументы:
	1. traj_type - тип траектории {sin polygon from_file}
	2. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller publish_path.launch traj_type:=2.0sin1.0
```

## 4.4 follow_path.launch
* Расположение rosbot_controller/launch/follow_path.launch
* Что делает: Запускает контроллер для следования rosbot по пути
* Пример использования
```
roslaunch rosbot_controller folow_path.launch
```

## 4.5 mppi_test.launch
* Расположение rosbot_controller/launch/mppi_test.launch
* Что делает: Запускает gazebo, спавнит rosbot, публикует траекторию
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
	3. traj_type - тип траектории {sin polygon from_file}
	4. move_plan - путь до файла с траекторией
	5. publish_path - true - публикует путь, false - не публикуеть путь
* Пример использования
```
roslaunch rosbot_controller mppi_test.launch traj_type:=2.5sin0.2 rviz:=true gui:=false 
```

