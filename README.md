ROSBot control mode development 

Инструкция по работе

# Docker 
## Install Guide
- Install dependecies
```
./dependencies.sh
```

- Build image
```
./build.sh
```

- Build & Run container
```
./run.sh
```

## Usage Guide
- Start container
```
docker start control
```

- Attach to container
```
docker attach control
```

- Open bash session in running container
```
docker exec -it control bash 
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
* Что делает: Запускает gazebo, спавнит rosbot, запукает model runners
* Аргументы:
	1. gui - запуск gui Gazebo
	2. rviz - запуск rviz
	3. use_nn_model - запуск нейосетевой модели rosbot
* Пример использования
```
roslaunch rosbot_controller run_simulation.launch traj_type:=2.5sin0.2 rviz:=true gui:=false use_nn_model:=true
```

## 4.2 follow_path.launch
* Расположение rosbot_controller/launch/follow_path.launch
* Что делает: публикует траекторию, запускает контроллер для следования rosobot по пути
* Аргументы:
	1. traj_type - тип траектории {sin polygon from_file}
	2. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller follow_path.launch traj_type:=2.5sin0.2
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