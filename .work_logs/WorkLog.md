Неделя 4 

    1. Формировать цели для робота в газебо и для модели независимо
        Решил разделить управление роботом и моделью робота на две разные ноды с одинковым кодом. 
        
        DONE 

        git commit -m "model runner and model follower runs ok"

    2. Научиться ездить по траектории из файла - Костя 

    3. Вставить кинематическую модель из Probability Robotics (стр 101) 

    4. Вставить динамическую модель от Лизы 

    5. Научиться телепортировать модель

    6. Интегрировать код Саши из ветки ShukDevelop

    7. Интегрировать MPC от ИВ



Неделя 3 - отправить модель по траектории

    1. вычиялеть управление по разным состояниям DONE
    
     


Неделя 2 - научиться нормально ездить по траектории с оценкой качества проезда

    1. Зарефакторить модель управления DONE

    2. Научиться генерить траекторию 

        Sin через начало координат - OK

    3. Научиться телепортировать робота - DONE

rosservice call /gazebo/set_model_state '{model_state: { model_name: rosbot, pose: { position: { x: 1.0, y: 0.0 ,z: 0.1 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'   

    ./teleport.sh


    4. Была идея сделать контроллер, учитывающий три точки траектории: предыдущую, текущую и следующую, тогда можно описывать по ним окружность, считать ее радиус и так связывать

        R = v/w 

        но потом понял, что таких идей будет очень много, и не ясно, наша ли это цель

    5. Перешел к идее сделать модель движения робота и параллельно публикать ее tf   DONE

        https://www.researchgate.net/publication/221333677_Simple_Path_Planning_Algorithm_for_Two-Wheeled_Differentially_Driven_2WDD_Soccer_Robots

        шаг 1. Вычислить смещение робота в системе координат, связанной с предыдущим положением робота

        шаг 2. перевести эту точку в неподвижную систему координат

        шаг 3. опубликать это в качестве tf нового положения робота 

        git commit -m "simple robot model implemented that really diverges from actual path" 


    6. Научиться вычислять ближайшую точку к траектории и стоимость траектории DONE

        git commit -m "path cost implemeted"

    7. Все теперь можно отправлять бота кататься по sin и квадрату

        roslaunch rosbot_controller folow_path.launch traj_type:=polygon



Неделя 1  - Отправить робота по траектории, управляя топиком cmd_vel


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


    Результаты тезисно:
        
        1. Робот ездит от промежуточной цели до промежуточной цели

            Надо бы сделать управление, которое понимало бы, что текущая промежуточная цель непоследняя и тогда робот бы там не останавливался (не замедлялся) 

            Либо у каждой цели должна быть еще скорость достижения цели

        2. Робот может крутиться на месте, тогда он сможет проехать по квадрату

            Надо давать промежуточную цель в виде точки с ориентацией ?

