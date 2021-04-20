import numpy as np


class RosbotKinematic():
    def __call__(self, batch):
        """Evaluates robot speeds in +1 dt time by given batch of current velocities, controls and dts

        Args:
            batch: np.ndarray of shape [batch_size, 5] where 5 for velocities [v, w], controls [v, w] and dt
        """
        return batch[:, 2:4]
