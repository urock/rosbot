## Open loop контроллер (публикация управления из файла)
Нода control_generator может публиковать управление из файла

Пример запуска
```
roslaunch rosbot_controller control_gen.launch control_mode:=from_file file_path:=$CONTROL_FILE_PATH launch_logger:=false 
```
Аргументы:
* control_mode:=from_file - обязательно указываем что управление публикуется
* file_path - путь до файла с управлением
* launch_logger - указываем не запускать ноду logger