from abc import ABC, abstractmethod


class Optimizer(ABC):
    @abstractmethod
    def set_reference_traj(self, ref_traj):
        """ Set reference traj to this

        Args:
            ref_traj: np.array 
        """
        pass

    @abstractmethod
    def update_state(self, state):
        """Set current state to this

        Args:
            state: current state of type State [x, y, yaw] 
        """
        pass

    @abstractmethod
    def next_control(self, goal_idx):
        """
        Calculates next best control 

        Args:
            goal_idx: idx of reference goal

        Return:
            next control of type Control - [v, w]
        """
        pass
