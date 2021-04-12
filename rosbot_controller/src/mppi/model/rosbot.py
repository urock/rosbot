import numpy as np

class Rosbot():
    def __call__(self, batch):
        velocities = batch[:, :2].view()
        controls = batch[:, 2:4].view()
        dts = batch[:, 4].view()
        return velocities * dts[0] + (controls - velocities)/2
