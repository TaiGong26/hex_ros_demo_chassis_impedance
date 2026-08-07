#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from abc import ABC, abstractmethod

import numpy as np


class ChassisDynamicsBase(ABC):
    """Base interface for chassis kinematics.

    ``calc_jac`` maps joint velocity to body twist, while ``calc_jac_inv``
    maps body twist to joint velocity. Their transposes provide the matching
    virtual-work effort mappings.
    """

    def __init__(self, chs_params: dict):
        self._chs_params = dict(chs_params)

    @abstractmethod
    def calc_jac(self, jnt_pos: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def calc_jac_inv(self, jnt_pos: np.ndarray) -> np.ndarray:
        raise NotImplementedError
