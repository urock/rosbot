### Управление генератором периодического управления

ROS нода `control_generator` позволяет создавать периодические последовательности управления

#### Пример управления линейной скоростью
**В этом случае робот ездит примерно по прямой линии**
```
roslaunch rosbot_controller control_gen.launch output_folder:=_8 Tmax:=30.0 period_lin:=10 v_max:=1.0 a_lin:=0.5
```
![GitHub Logo](/docs/images/linear_example.png)

#### Пример управления угловой скоростью
**В этом случае робот крутится на месте**
```
roslaunch rosbot_controller control_gen.launch output_folder:=_9 Tmax:=30.0 period_ang:=10 v_max:=0.0 w_min:=0.0 w_max:=1.0 
```
![GitHub Logo](/docs/images/angular_example.png)

#### Пример управления линейной и угловыми скоростями скоростью
**В этом случае робот выписывает интересные спирали**
```
roslaunch rosbot_controller control_gen.launch output_folder:=_13 Tmax:=30.0 period_lin:=10 period_ang:=10 v_max:=1.5 w_min:=-3.0 w_max:=3.0 a_ang:=2.0
```
![GitHub Logo](/docs/images/linear_and_angular_examples.png)