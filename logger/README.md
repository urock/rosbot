# Logger


# logger_node
Нода для логирования состояния робота, управления и времени.
Состояние робота представлено вектором из 5 элементов - [X, Y, YAW, V, W], где:
	* X, Y - координаты
	* YAW - угол рысканья (курса)
	* V - линейная скорость
	* W - угловая скорость

Управление представлено вектором из 2 элементов - [X, YAW]
	* X - управление линйной скоростью
	* YAW - управление угловой скоростью


## Принцип работы

Кажды раз когда публикуется tf сообщение parent_frame - robot_frame, логируется состояние робота (state.csv), и кинематической модели (kinetic_state.csv) текущее управление (control.csv), и время(time.csv). 

## Как запустить

```bash
roslaunch logger logger.launch output_folder:=/$TRAJ_NAME

```
*$TRAJ_NAME* - имя выходной директории для собраных данных
#### Аргументы logger.launch
* output_file - имя выходной папки для собраных данных
  (defaut value = "")
* output_folder - путь до выходной директории
  (defaut value = "../logger/output_data")
* show_plots - если true показывает графики по завершению работы ноды 
  (defaut value = False)
* track_time - если true учитывает время запуска в имени выходной папки
  (defaut value = False)
* parent_frame - 
  (defaut value = "odom")
* robot_frame - 
  (defaut value = "base_link")
* kinetic_model_frame - 
  (defaut value = "model_link")

## Выходнеы файлы с данными

* **state.csv** - файл с состоянием робота
* **kinetic_state.csv** - файл с состоянием кинематической модели робота
* **control.csv** - файл с управлением
* **time.csv** - файл с временем

## Графики

* **general_graph.png** - график заданной траектории и пройденного роботом пути
* **state.png** - 4 графика, для робота и кинематической модели совместные графики пройденного пути Y(x), а также X(t), Y(t), YAW(t)
* **velocitites_and_control.png** - графики 

# create_graphs
Утилита для построения графиков, как у ноды logger, по готовым данным. Графики воможно масштабировать, сохранять.

## Как запустить

```bash
python3 create_graphs.py -folder_path=$TRAJ_PATH

```
*$TRAJ_PATH* - путь до директории с данными
