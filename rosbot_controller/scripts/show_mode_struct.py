
import sys
import numpy as np
import nnio
import onnx

path = "/home/user/catkin_ws/src/rosbot_controller/nets/rosbot_gazebo9_2d_model.onnx"
# model = nnio.ONNXModel(path)
mode = onnx.load(path)
# model is an onnx model
graph = model.graph
# graph inputs
for input_name in graph.input:
    print(input_name)
# graph parameters
for init in graph.init:
    print(init.name)
# graph outputs
for output_name in graph.output:
    print(output_name)
# iterate over nodes
for node in graph.node:
    # node inputs
    for idx, node_input_name in enumerate(node.input):
        print(idx, node_input_name)
    # node outputs
    for idx, node_output_name in enumerate(node.output):
        print(idx, node_output_name)