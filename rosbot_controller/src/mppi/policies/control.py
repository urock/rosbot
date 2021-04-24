import numpy as np


def calc_softmax_seq(batch_costs: np.array, control_batch: np.ndarray):
    """Calculate control using softmax function.

    Args:
        control_batch: np.ndarray of shape [batch_size, time_steps, 2]
        costs: np.array of shape [batch_size]
    """
    T = 0.15  # Temperature

    batch_costs = batch_costs - np.min(batch_costs)
    exponents = np.exp(-1 / T * (batch_costs))
    softmaxes = exponents / np.sum(exponents)  # -> shape = [batch_size]

    control = (control_batch * softmaxes[:, np.newaxis, np.newaxis]).sum(axis=0)
    return control


def find_min_seq(batch_costs: np.array, control_batch: np.ndarray):
    best_idx = np.argmin(batch_costs, axis=0)
    return control_batch[best_idx]
