from abc import ABC, abstractmethod
import rospy


class Optimizer(ABC):

    @abstractmethod
    def set_reference_traj(self, ref_traj):
        """Set reference traj to this.

        Args:
            ref_traj: np.array of points [x, y, yaw]
        """

    @abstractmethod
    def update_state(self, state):
        """Set current state to this.

        Args:
            state: current state of type State [x, y, yaw, v, w]
        """

    @abstractmethod
    def next_control(self, goal_idx):
        """Calculate next best control.

        Args:
            goal_idx: idx of reference goal

        Return:
            next control of type Control - [v, w]
        """
