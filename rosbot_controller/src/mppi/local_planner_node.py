#!/usr/bin/env python3

import nnio
import cProfile
import pstats
import io
import signal

import rospy

from policies.costs import triangle_cost, nearest_cost
from policies.control import calc_softmax_seq, find_min_seq
from policies.metrics import mean_dist_metric
from models.rosbot import RosbotKinematic
from optimizers.mppic import MPPIController
from local_planner import LocalPlanner
from robot import Odom



pr = cProfile.Profile(timeunit=0.00)


def main():
    signal.signal(signal.SIGINT, handler)

    pr.enable()
    start_planner()
    pr.disable()


def start_planner():
    rospy.init_node('planner', anonymous=True, disable_signals=True)
    model_path = rospy.get_param('~mppic/model_path', None)

    model = nnio.ONNXModel(model_path)

    optimizer = MPPIController(model, nearest_cost, calc_softmax_seq)
    odom = Odom()
    mppic = LocalPlanner(odom, optimizer, mean_dist_metric)

    mppic.start()
    rospy.spin()


def handler(signum, frame):
    rospy.signal_shutdown("Local Planner shutdown")
    rospy.loginfo("***************** Profiling *****************\n")
    profile()


def profile():
    s = io.StringIO()
    sortby = 'cumtime'
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats(20)
    print(s.getvalue())

if __name__ == '__main__':
    main()
