## Утилиты построения графиков после проезда
Утилита create_graphs парсит собранные csv данные, и строит по ним графики

Пример запуска для одного проезда
```
python3 create_graphs.py -folder_path $DATA_PATH
```
, где
$DATA_PATH - путь до директории с .csv файлами


Пример запуска для одного проезда
```
python3 create_graphs.py -folder_path $DATA_FOLDER_PATH T-group True
```
, где
$DATA_FOLDER_PATH - путь до директории, в котрой нахдятся папки с .csv данными

Если #DATA_PATH - ~/output_data/tests/test_1
то DATA_FOLDER_PATH - ~/output_data/tests