import numpy as np


class Rosbot():
    def __call__(self, batch):
        """Evaluates robot speeds in +1 dt time by given batch of current velocities, controls and dts

        Args:
            batch: np.array of shape [batch_size, 5] where 5 for velocities [v, w], controls [v, w] and dt
        """
        velocities = batch[:, :2].view()
        controls = batch[:, 2:4].view()
        dts = batch[:, 4].view()
        return velocities * dts[0] + (controls - velocities)/2
