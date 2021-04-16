#!/usr/bin/env python3

import nnio
import cProfile
import pstats
import io
import signal

import rospy

from utils.optimization.losses import sum_loss, triangle_loss, nearest_loss
from utils.optimization.policies import calc_softmax_seq, find_min_seq
from models.rosbot import RosbotKinematic

from robot import Odom
from mppic import MPPIController
from local_planner import LocalPlanner

pr = cProfile.Profile(timeunit=0.00)


def main():
    signal.signal(signal.SIGINT, handler)

    pr.enable()
    start_planner()
    pr.disable()

def start_planner():
    rospy.init_node('mppic', anonymous=True, disable_signals=True)

    model_path = rospy.get_param('~model_path', None)
    model = nnio.ONNXModel(model_path)
    # model = RosbotKinematic()

    optimizer = MPPIController(model, nearest_loss, calc_softmax_seq)
    odom = Odom()
    mppic = LocalPlanner(odom, optimizer)

    mppic.start()
    rospy.spin()


def handler(signum, frame):
    rospy.signal_shutdown("Local Planner shutdown")
    rospy.loginfo("***************** Profiling *****************\n")

    s = io.StringIO()
    sortby = 'cumtime'
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats(20)
    print(s.getvalue())

if __name__ == '__main__':
    main()
