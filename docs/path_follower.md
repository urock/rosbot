## Closed loop path follower контроллер следования по траектории

Пример запуска
```
roslaunch rosbot_controller follow_path.launch traj_type:=$TRAJ_TYPE
```
Аргумент:
* traj_type - тип траектории $TRAJ_TYPE={sin, spiral, polygon, from_file}
Если traj_type == from_file
* move_plan - путь до файла с траекторией (список точек)

пример файла траектории
**move_plan.txt**
```
0.1 2.0
2.1 1.9
2.0 -0.1
0.0 0.0
```

В данной лаунч файле запускается две ноды. Первая, publish_path node - пуликует заданный путь. Вторая path_follower node является реализацией простого контроллера для следования по пути.

Пример отдельного запуска publish_path
```
roslaunch rosbot_controller publish_path.launch traj_type:=$TRAJ_TYPE
```
Аргумент:
* traj_type - тип траектории $TRAJ_TYPE={sin, spiral, polygon, from_file}
Если traj_type == from_file
* move_plan - путь до файла с траекторией (список точек)