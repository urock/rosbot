from abc import ABC, abstractmethod

class Optimizer(ABC):
    @abstractmethod
    def set_reference_traj(self, ref_traj):
        pass

    @abstractmethod
    def update_state(self, state):
        pass

    @abstractmethod
    def next_control(self, goal_idx):
        return control
