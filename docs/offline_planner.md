## Планирование оптимального управления 

Расчет open loop управления методом PSO для следования в заданную точку с объездом препятвий, заданных в виде кругов на плоскости. 

```
docker start gazebo-control
docker attach gazebo-control
cd /home/user/catkin_ws/src/rosbot_controller/src/offline_planner
python3 offline_planner.py -it 150 -m ~/catkin_ws/src/rosbot_controller/nets/rosbot_gazebo9_2d_model.onnx -o ~/control1.txt
```

Аргументы:
	
* it - количетсво итераци работы алгоритма
* m - путь до onnx файла нейросетевой модели
* o - путь до выходного файла для записи результата расчета управления


Внутри `offline_planner.py` захардкожено в функции main: 
 * начальное состояние робота `current_state`
 * цель следования `goal`
 * конфигурация препятствий `obstacles`


### Тест расчитанного управления в Gazebo

```
roslaunch rosbot_controller offline_planner_test.launch file_path:=/home/user/control1.txt output_folder:=/c3
```

Аргументы:
	
* file_path - путь к файлу с расчитанным управлением
* output_folder - путь к директории для сохранения графиков и csv таблиц (относительно /logger/output_data)



### Красивая визуализация нескольких проездов

После того как собрано N проездов (то есть N раз запустили `offline_planner_test.launch` с разными  директориями для сбора данных) можно построить график по всем проездам. Для этого нужно в отдельную директорию перенести папки с проездами по которым нужно строить график и запустить команду. 

```
python3 create_graphs.py -folder_path /home/user/catkin_ws/src/logger/output_data -group True
```

Аргументы:
	
* folder_path -  абсолютный путь к директории, содержащей директории с данными после проездов
* group = True - флаг, означающий что строится график для группы траекторий

Пример графиков

![PSO_example_0](/docs/images/PSO_plot_example_0.png)


![PSO_example_1](/docs/images/PSO_plot_example_1.png)