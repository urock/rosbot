## Детальное описание launch файлов

### run_simulation.launch
Что делает: Запускает gazebo, спавнит rosbot, запукает model runners

Аргументы:
	
* gui - запуск gui Gazebo
* rviz - запуск rviz
* use_nn_model - запуск нейосетевой модели rosbot
* Пример использования
```
roslaunch rosbot_controller run_simulation.launch rviz:=true gui:=true use_nn_model:=false
```

### control_gen.launch
Что делает: Запускает gazebo, спавнит rosbot, запукает model runners

Аргументы:
	
* Tmax          - длительность управления
* period_lin    - период изменения линейной скорости
* period_ang    - период изменения угловой скорости
* v_min         - максимальная линейная скорость
* v_max         - минимальная линейная скорость
* w_min         - максимальная угловая скорость
* w_max         - минимальная угловая скорость
* a_lin         - линейное ускорение (задавать только положительные числа)
* a_ang         - угловое ускорение (задавать только положительные числа)
* launch_logger - флаг запуска логгера, true по-умолчанию
* output_folder - выходная директория логгера 

* Пример использования
```
roslaunch rosbot_controller control_gen.launch output_folder:=_8 Tmax:=30.0 period_lin:=10 v_max:=1.0 a_lin:=0.5
```

### 4.2 follow_path.launch
* Расположение rosbot_controller/launch/follow_path.launch
* Что делает: публикует траекторию, запускает closed loop simple контроллер для следования rosobot по пути
* Аргументы:
	1. traj_type - тип траектории {sin polygon from_file}
	2. move_plan - путь до файла с траекторией
* Пример использования
```
roslaunch rosbot_controller follow_path.launch traj_type:=2.5sin0.2
```



### 4.4 logger.launch


### 4.5 mppi_test.launch
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

