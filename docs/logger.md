## Логирование состояния робота и управления

Создан ROS модуль, обеспечивающий логгирование координат робота и управления

Пример запуска
```
roslaunch logger logger.launch output_file:=test_nn_1
```
  
output_folder - путь до выодной директории   
output_file - имя папки с данными внутри выходной директории
parent_frame - имя статичного фрейма
robot_frame - имя фрейма робота
kinetic_model_frame - имя фрейма  кинематической модели
track_time - флаг, учитывать время в названии директории
show_plots - флаг, показывать выходные графики

### Визуализация моделей - кинематическая и нейросетевая

Для визуализации кинематической модели

```
roslaunch rosbot_controller spawn_kinematic_model.launch
```

Для визуализации нейросетевой модели

```
roslaunch rosbot_controller spawn_nn_model.launch nn_model_path:=$ONNX_MODEL_PATH
```
Аргументы:
* nn_model_path - путь до .onnx модели 

$ONNX_MODEL_PATH - путь до .onnx модели 
