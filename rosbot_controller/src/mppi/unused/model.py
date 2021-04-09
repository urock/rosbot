#!/usr/bin/env python

from abc import ABC, abstractmethod

class Model(ABC):
    @abstractmethod
    def __call__(self, state, control, dt):
        """
        Returns robot state assuming that control has been active for dt seconds
        """
