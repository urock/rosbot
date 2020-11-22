ROSBot control mode development 

Задача 1  - Отправить робота по траектории, управляя топиком cmd_vel

    ```
    cd ~/work/rosbot/catkin_ws
    source devel/setup.zsh
    roslaunch rosbot2 urock.launch
    rosrun simple_vel_controller simple_vel.py
    ```

    Tasks
    
        1. Запустить rosbot2 в пустом мире      DONE
        
            с путым миром real time factor упал в 10 раз

            сдела компю maze.world (в нем был параметр real_time_factor = 1) без стен - empty.world
        
        2. Подготовить нормальеый rviz          DONE 
        

            git commit -m "rosbot goes to /move_base_simple/goal from rviz in empty world"
        
        3. Визуализировать base_link            DONE

            git commit -m "path_viz added"

        4. Организовать очередь команд в simple_vel.py  

            Нашел багу - если ехать налево, то движение оказывается неусточивым
                - бага была в том, что я направление на цель вычислял один раз в момент получения цель, а надо на каждой итерации 

