#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np

from .base import ChassisDynamicsBase


class TriggerADynamics(ChassisDynamicsBase):
    """Extension point for Trigger A chassis kinematics.

    ``wheel_distance`` is supplied directly through ``chs_wheel_distance``
    instead of being derived from a track width and wheel base.
    """

    def __init__(self, chs_params: dict):
        super().__init__(chs_params)
        beta = np.array([np.pi / 3, -np.pi / 3, np.pi])
        s, c = np.sin(beta), np.cos(beta)
        self.__jac_inv = -1.0 / self._chs_params["wheel_radius"] * np.array([
            [-s[0], c[0], self._chs_params["wheel_distance"]],
            [-s[1], c[1], self._chs_params["wheel_distance"]],
            [-s[2], c[2], self._chs_params["wheel_distance"]],
        ])
        self.__jac = np.linalg.pinv(self.__jac_inv)

    def calc_jac(self, jnt_pos: np.ndarray) -> np.ndarray:
        return self.__jac

    def calc_jac_inv(self, jnt_pos: np.ndarray) -> np.ndarray:
        return self.__jac_inv
