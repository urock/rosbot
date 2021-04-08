# Rosbot control mode development 

Репозиторий инструментов работы с Rosbot в ROS Melodic для обучения нейросетевой диманической модели робота и разработки MPC контроллера управления. 

## Введение

В репозитории созданы следующие инструменты:

1. Управление дифференциальным роботом
	* генератор периодических последовательноcтей управления
	* контроллер следования по траектории 
2. Логирование состояния робота и управления
3. Утилиты построения графиков после проезда
4. Средства автоматической сборки данных 

## Docker 

Рекомендуется работать через Docker. 

Для работы в Gazebo сборка образа и запуск контейнера осуществляется из директории docker_gazebo: создается `gazebo-control-image` образ и запускается `gazebo-control` контейнер. 

Для работы на реальном роботе сборка образа и запуск контейнера осуществляется из директории docker_robot: создается `robot-control-image` образ и запускается `robot-control` контейнер. **to be done**

### Install Guide
- Install docker  https://docs.docker.com/engine/install/ubuntu/
- Docker post install steps https://docs.docker.com/engine/install/linux-postinstall/
- Install dependecies
```
./$docker_dir/dependencies.sh		# установка зависимостей на локальную машину
./$docker_dir/build.sh				# Build image 
./$docker_dir/run.sh				# Create & Run container
```
где docker_dir = `docker_gazebo, docker_robot`

### Usage Guide
```
docker start $container_name 		 # Start container
docker attach $container_name 		 # Attach to container
docker exec -it $container_name bash # Open bash session in running container 
```
где container_name - `gazebo-control, robot_control`

## 2. Компиляция 

```
docker attach $container_name
cd catkin_ws
catkin build
source devel/setup.zsh  # необходимо делать только во время первого запуска контейнера
```

## 3. Сбор данных
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

### 4.1 run_simulation.launch
Что делает: Запускает gazebo, спавнит rosbot, запукает model runners

Аргументы:
	
* gui - запуск gui Gazebo
* rviz - запуск rviz
* use_nn_model - запуск нейосетевой модели rosbot
* Пример использования
```
roslaunch rosbot_controller run_simulation.launch rviz:=true gui:=false use_nn_model:=false
```

### 4.2 follow_path.launch
* Расположение rosbot_controller/launch/follow_path.launch
* Что делает: публикует траекторию, запускает контроллер для следования rosobot по пути
* Аргументы:
	1. traj_type - тип траектории {sin polygon from_file}
	2. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller follow_path.launch traj_type:=2.5sin0.2
```

### 4.3 control_gen.launch
* Расположение 
* Пример использования
```
roslaunch rosbot_controller control_gen.launch
```


### 4.4 mppi_test.launch
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
