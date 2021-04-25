#!/usr/bin/env python3

import nnio
import cProfile
import pstats
import io
import signal

import rospy

from policies.costs import TriangleCost, NearestCost
from policies.control import calc_softmax_seq, find_min_seq
from policies.metrics import mean_dist_metric

from optimizers.mppic_generator import MPPICGenerator
from optimizers.mppic_optimizer import MPPICOptimizer

from local_planner import LocalPlanner
from modules.robot import Odom
from modules.controller import Controller
from modules.goal_handler import GoalHandler
from modules.path_handler import PathHandler
from modules.metric_handler import MetricHandler


pr = cProfile.Profile(timeunit=0.00)
FUNCTION_PRINT_COUNT = 40


def main():
    signal.signal(signal.SIGINT, handler)

    pr.enable()
    start_planner()
    pr.disable()


def start_planner():
    rospy.init_node("planner", anonymous=True, disable_signals=True)

    model_path = rospy.get_param("~mppic/model_path", None)
    model = nnio.ONNXModel(model_path)
    cost = NearestCost(3)
    control_generator = MPPICGenerator(model)
    optimizer = MPPICOptimizer(control_generator, cost, calc_softmax_seq)
    odom = Odom()
    controller = Controller()
    goal_handler = GoalHandler()
    path_handler = PathHandler()
    metric_handler = MetricHandler(mean_dist_metric)
    mppic = LocalPlanner(optimizer, odom, controller, goal_handler, path_handler, metric_handler)

    mppic.start()
    rospy.spin()


def handler(signum, frame):
    rospy.signal_shutdown("Local Planner shutdown")
    rospy.loginfo("***************** Profiling *****************\n")
    profile()


def profile():
    s = io.StringIO()
    sortby = "cumtime"
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats(FUNCTION_PRINT_COUNT)
    print(s.getvalue())


if __name__ == "__main__":
    main()
