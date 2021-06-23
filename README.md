# Rosbot control mode development 

Репозиторий инструментов работы с Rosbot в ROS Melodic для управления роботом в Gazebo, сбора данных для обучения нейросетевой диманической модели робота и разработки MPC контроллера управления. 

## Введение

В репозитории созданы следующие инструменты:

0. Запуск симуляции в Docker (см ниже)
1. Управление дифференциальным роботом
	* [Генератор периодических последовательноcтей управления](/docs/control_gen.md)
	* [Open loop контроллер (публикация управления из файла)](/docs/open_loop.md) 
	* [Closed loop path follower контроллер следования по траектории](/docs/path_follower.md)
	* [Closed loop MPPI контроллер](/docs/mppi.md)
2. [Логирование состояния робота и управления](/docs/logger.md)
	* Визуализация моделей - кинематическая и нейросетевая 
3. [Утилиты построения графиков после проезда](/docs/create_graphs.md)
4. [Средства автоматической сборки данных](/docs/data_collect.md) 
6. [Планирование оптимального управления](/docs/offline_planner.md)

## Docker 

Рекомендуется работать через Docker. 

Для работы в Gazebo сборка образа и запуск контейнера осуществляется из директории docker_gazebo: создается `gazebo-control-image` образ и запускается `gazebo-control` контейнер. 


### Install Guide
- Install docker  https://docs.docker.com/engine/install/ubuntu/
- Docker post install steps https://docs.docker.com/engine/install/linux-postinstall/
```
./docker_gazebo/dependencies.sh		# установка зависимостей на локальную машину
./docker_gazebo/build.sh			# Build image 
./docker_gazebo/run.sh				# Create & Run container
```

### Usage Guide
```
docker start gazebo-control 		 
docker attach gazebo-control   		 
```

## Компиляция и запуск симуляции

```
docker attach $container_name
cd catkin_ws
catkin build
source devel/setup.zsh  # необходимо делать только во время первого запуска контейнера
roslaunch rosbot_controller run_simulation.launch rviz:=true gui:=true
```

Примеры работы с определенными инструментами смотри в соответствующих разделах оглавления.

Также существуют [детальные описания launch файлов](/docs/launch_files.md). 









