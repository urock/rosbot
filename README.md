ROSBot control mode development 

Задача 1  - Отправить робота по траектории, управляя топиком cmd_vel

    ```
    cd ~/work/rosbot/catkin_ws
    source devel/setup.zsh
    roslaunch rosbot2 urock.launch
    roslaunch simple_vel_controller vel_control.launch
    ```

    Tasks
    
        1. Запустить rosbot2 в пустом мире      DONE
        
            с путым миром real time factor упал в 10 раз

            сдела компю maze.world (в нем был параметр real_time_factor = 1) без стен - empty.world
        
        2. Подготовить нормальеый rviz          DONE 
        

            git commit -m "rosbot goes to /move_base_simple/goal from rviz in empty world"
        
        3. Визуализировать base_link            DONE

            git commit -m "path_viz added"

        4. Организовать очередь команд в simple_vel.py  DONE  

            Нашел багу - если ехать налево, то движение оказывается неусточивым
                - бага была в том, что я направление на цель вычислял один раз в момент получения цель, а надо на каждой итерации 

            git commit -m "goal queue added to simple vel controller"

        5. Добавить pub and sub nav_msg Path        DONE

            git commit -m "simple xy path topic added"