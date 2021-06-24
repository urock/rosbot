# Model Predictive Path Integral

## Set Paths
To set paths set "paths" param in config/paths.yaml. Only two types are avaliable now (sin, polygon). Each type has its own args e.g:

```
paths : [
    {
        'type': 'sin',
        'args': {'step': 0.1, 'amplitude': 1.0, 'freq' : 0.1 }
    },
    {
        'type': 'polygon',
        'args': {'step': 0.1, 'edges': [[0.0, 0.0], [0.0, 5.0], [5.0, 5.0], [5.0, 0.0] ] }
    }
]
```

You can then switch between them in real time using service call ```/next_path```

## Set obstacles
To set obstacles set "obstacles" param in config/mppi.yaml in format [x, y, radius], e.g:

```
obstacles: [              
              [1, 0,  0.3],
              [2.5, 0.55,  0.3]
           ]
```


## Run MPPI
```
roslaunch rosbot_controller mppi.launch
```


## Run next path 
After running MPPI call the service:
``` 
rosservice call /next_path
```

## Change MPPI params
You can change MPPI params in real time using dynamic_reconfigure in rqt. Default params are set in cfg/MPPI.cfg. 

## MPPI params


| Parameter       | Type   | Definition                                                                         |
| --------------- | ------ | ---------------------------------------------------------------------------------- |
| traj_vis_step   | int    | Step for trajectories which will be visualized                                     |
| iter_count      | int    | Number of MPPI iterations                                                          |
| traj_lookahead  | double | Global trajectory lookahead in meters                                              |
| batch_size      | int    | Count of generated on each iteration trajectories                                  |
| time_steps      | int    | Number of time steps in recursive mppi propagation                                 |
| model_dt        | double | Time step dt                                                                       |
| v_std           | double | Linear velocity std of gaussian distrubution                                       |
| w_std           | double | Angular velocity std of gaussian distrubution                                      |
| limit_v         | double | Limit on linear velocity control                                                   |
| limit_w         | double | Limit on angular velocity control                                                  |
| temperature     | double | Parameter of MPPI which influence selectiveness of generated trajectories          |
| goal_weight     | double | Weight of goal component of cost function                                          |
| goal_power      | int    | Power of goal component of cost function                                           |
| reference_weigh | double | Weight of reference component of cost function                                     |
| reference_power | int    | Power of reference component of cost function                                      |
| obstacle_weight | double | Weight of obstacle component of cost function                                      |
| obstacle_power  | int    | Power of obstacle component of cost function                                       |
| stop_robot      | bool   | Stop robot                                                                         |
| wait_full_step  | bool   | Start next iteration only after time multiple of dt passed                         |
| visualize       | bool   | Visualization of trajectories, considerable reference trajectory, path passed etc. |