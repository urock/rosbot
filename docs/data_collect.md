## Средства автоматической сборки данных 

```bash
cd catkin_ws_path
cd src/rosbot/rosbot_controller/src/ 
./test.zsh -t="5.0sin0.1 -5.0sin0.2 3.0sin3.0 2.0sin1.5" -v="1.5 2.5 3.5 4.5" -w="0.5 1.0 2.5 3.0"
```
* -t - список траекторий
* -v - список линейных скоростей
* -w - список угловых скоростей
Данные будут сохранены в *logger/output_data/*


или автоматическое средство сбора случайно сгенерированных последовательностей управления

```bash
cd catkin_ws_path
cd src/rosbot/rosbot_controller/src/ 
./colect_data.zsh
```
