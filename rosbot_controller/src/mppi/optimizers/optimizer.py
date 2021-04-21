from abc import ABC, abstractmethod
import rospy


class Optimizer(ABC):
    def __init__(self, model, cost, next_control_policy):
        self.model = model
        self.calc_costs = cost
        self.next_control_policy = next_control_policy 
        self.traj_lookahead = int(rospy.get_param('~optimizer/traj_lookahead', 7))

    @abstractmethod
    def set_reference_traj(self, ref_traj):
        """ Set reference traj to this

        Args:
            ref_traj: np.array 
        """

    @abstractmethod
    def update_state(self, state):
        """Set current state to this

        Args:
            state: current state of type State [x, y, yaw] 
        """

    @abstractmethod
    def next_control(self, goal_idx):
        """
        Calculates next best control 

        Args:
            goal_idx: idx of reference goal

        Return:
            next control of type Control - [v, w]
        """
