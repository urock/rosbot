#!/bin/bash

rosservice call /gazebo/set_model_state '{model_state: { model_name: rosbot, pose: { position: { x: 1.0, y: 0.0 ,z: 0.1 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'