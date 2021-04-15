import numpy as np


def calc_softmax_seq(batch_losses: np.array, control_batch: np.ndarray):
    """ Calculate control using softmax function

    Args:
        control_batch: np.ndarray of shape [batch_size, time_steps, 2]
        losses: np.array of shape [batch_size]
    """
    T = 0.15 # Temperature

    batch_losses = batch_losses - np.min(batch_losses)
    exponents = np.exp(-1/T * (batch_losses))
    softmaxes = exponents / np.sum(exponents) # -> shape = [batch_size]

    control = (control_batch * softmaxes[:, np.newaxis, np.newaxis]).sum(axis=0)
    return control 

def find_min_seq(batch_losses: np.array, control_batch: np.ndarray):
    best_idx = np.argmin(batch_losses, axis=0)
    return control_batch[best_idx] 

