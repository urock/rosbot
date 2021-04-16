import rospy

from utils.losses import sum_loss, order_loss, nearest_loss
from utils.policies import calc_softmax_seq, find_min_seq
from models.rosbot import RosbotKinematic


def main():
    rospy.init_node('mppic', anonymous=True)

    freq = int(rospy.get_param('~cmd_freq', 30))
    model_path = rospy.get_param('~model_path', None)

    batch_size = int(rospy.get_param('~batch_size', 100))
    time_steps = int(rospy.get_param('~batch_size', 50))
    iter_count = int(rospy.get_param('~batch_size', 1))

    v_std = rospy.get_param('~v_std', 0.1)
    w_std = rospy.get_param('~w_std', 0.1)

    limit_v = rospy.get_param('~limit_v', 0.5)
    limit_w = rospy.get_param('~w_std', 0.7)

    desired_v = rospy.get_param('~desired_v', 0.5)
    traj_lookahead = int(rospy.get_param('~desired_v', 7))

    loss = sum_loss
    control_policie = calc_softmax_seq

    model_path = model_path
    model = nnio.ONNXModel(model_path)
    # model = RosbotKinematic()

    optimizer = MPPIControler(loss, control_policie,
                              freq, v_std, w_std, limit_v, limit_w, desired_v,
                              traj_lookahead,
                              iter_count, time_steps, batch_size, model)

    map_frame = rospy.get_param('~map_frame', "odom")
    base_frame = rospy.get_param('~base_frame', "base_link")
    odom = Odom(map_frame, base_frame)

    goal_tolerance = rospy.get_param('~v_std', 0.2)
    goals_interval = rospy.get_param('~w_std', 0.1)
    mppic = LocalPlanner(odom, optimizer, goal_tolerance, goals_interval)

    mppic.start()
    rospy.spin()


if __name__ == '__main__':
    main()
