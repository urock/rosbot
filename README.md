Репозиторий для работы с дифференциальным роботом в ROS Melodic

# 1. Установка

```
git clone git@github.com:urock/rosbot.git
cd rosbot
git submodule update --init --recursive
```

# 5. Работа через Docker

```
cd docker

# собрать докер образ
./build_docker_image.bash 

# создает котейнер и заходит в его терминал, если выйти из терминала, то контейнер продолжит работу
./create_docker_container.bash 

# зайти в терминал, созданного ранее докер контейнера 
./enter_container.bash 

```

После создания образа и первого входна в контейнер:
```
./build_docker_image.bash 
./create_docker_container.bash 

cd /home/user/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_init_workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

выйти из контейнера (exit) и зайти заново 
```
./enter_container.bash 
roslaunch rosbot_controller run_simulation.launch traj_type:=2.5sin0.2 rviz:=true gui:=true
```


# 2. Запуск симулятора
открыть терминал
```bash
cd catkin_ws_path
source devel/setup.zsh
roslaunch rosbot_controller run_simulation.launch traj_type:={sin, polygon} use_nn_model:={true, false}
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
	5. move_plan - запуск нейосетевой модели rosbot
* Пример использования
```
roslaunch rosbot_controller run_simulation.launch traj_type:=2.5sin0.2 rviz:=true gui:=false use_nn_model:=true
```

## 4.2 spawn_rosbot.launch
* Расположение rosbot2/launch/spawn_rosbot.launch
* Что делает: Запускает gazebo, спавнит rosbot
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
* Пример использования
```
roslaunch rosbot2 spawn_rosbot.launch rviz:=true gui:=false 
```

## 4.3 spawn_kinematic_model.launch
* Расположение rosbot2/launch/spawn_kinematic_model.launch
* Что делает: спавнит кинематическую модель rosbot
* Аргументы:
	1. robot_frame - TF Frame модели
	2. cmd_topic - топик с командами управления для модели
* Пример использования
```
roslaunch rosbot_controller spawn_kinematic_model.launch 
```

## 4.4 spawn_nn_model.launch
* Расположение rosbot2/launch/spawn_nn_model.launch
* Что делает: спавнит нейросетевую модель rosbot
* Аргументы:
	1. nn_model_path - путь до onnx модели
	2. cmd_topic - запуск rviz
	3. parent_frame - Статичный TF Frame (odom)
	4. robot_frame - TF Frame модели
	5. cmd_freq - частота обновления модели
* Пример использования
```
roslaunch rosbot_controller spawn_nn_model.launch 
```

## 4.5 publish_path.launch
* Расположение rosbot_controller/launch/publish_path.launch
* Что делает: публикует траекторию для rosbot
* Аргументы:
	1. traj_type - тип траектории {sin polygon from_file}
	2. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller publish_path.launch traj_type:=2.0sin1.0
```

## 4.6 follow_path.launch
* Расположение rosbot_controller/launch/follow_path.launch
* Что делает: Запускает контроллер для следования rosbot по пути
* Пример использования
```
roslaunch rosbot_controller folow_path.launch
```

## 4.8 mppi_test.launch
* Расположение rosbot_controller/launch/mppi_test.launch
* Что делает: Запускает gazebo, спавнит rosbot, публикует траекторию
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
	3. traj_type - тип траектории {sin polygon from_file}
	4. move_plan - путь до файла с траекторией
	5. publish_path - true - публикует путь, false - не публикуеть путь
	6. nn_model_path - путь до нейросетевой модели
* Пример использования
```
roslaunch rosbot_controller mppi_test.launch traj_type:=2.5sin0.2 rviz:=true gui:=false 
```




# 6. Работа в tmux 

* управляющая последовательность: CS = 'Ctrl + A'
* создать вкладку
```
# внизу
CS + '-'
# справа
CS + '- shift'   
```
* переход между вкладками: `CS + ' стрелки вправо, влево, вниз, вверх
* убить вкладку: 'CS + x'