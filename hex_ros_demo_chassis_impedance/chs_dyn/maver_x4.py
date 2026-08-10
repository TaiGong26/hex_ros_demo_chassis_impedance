#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np

from .base import ChassisDynamicsBase

_CHS_DOF = 8
_WHEEL_IDX = [0, 2, 4, 6]
_YAW_IDX = [1, 3, 5, 7]


class MaverX4Dynamics(ChassisDynamicsBase):
    """Kinematics for the four-steering-wheel Maver X4 chassis."""

    def __init__(self, chs_params: dict):
        super().__init__(chs_params)
        self._chs_params["bias_inv"] = 1.0 / self._chs_params["bias"]
        self._chs_params["wheel_radius_inv"] = (
            1.0 / self._chs_params["wheel_radius"])
        self._chs_params["wheel_distance"] = 0.5 * np.ones(
            4) * np.sqrt(self._chs_params["track_width"]**2 +
                         self._chs_params["wheel_base"]**2)
        beta = np.arctan2(self._chs_params["track_width"],
                          self._chs_params["wheel_base"])
        self._chs_params["beta"] = np.array(
            [beta, np.pi - beta, beta - np.pi, -beta], dtype=np.float64)

    def calc_jac(self, jnt_pos: np.ndarray) -> np.ndarray:
        return np.linalg.pinv(self.calc_jac_inv(jnt_pos))

    def calc_jac_inv(self, jnt_pos: np.ndarray) -> np.ndarray:
        yaw = np.asarray(jnt_pos[_YAW_IDX], dtype=np.float64).reshape(4)
        sin_theta = np.sin(yaw)
        cos_theta = np.cos(yaw)
        sin_theta_beta = np.sin(yaw - self._chs_params["beta"])
        cos_theta_beta = np.cos(yaw - self._chs_params["beta"])

        mat_wheel = np.column_stack(
            (cos_theta, sin_theta, self._chs_params["wheel_distance"] *
             sin_theta_beta)) * self._chs_params["wheel_radius_inv"]
        mat_yaw = np.column_stack(
            (-sin_theta, cos_theta,
             self._chs_params["wheel_distance"] * cos_theta_beta -
             self._chs_params["bias"])) * self._chs_params["bias_inv"]

        jac_inv = np.empty((_CHS_DOF, 3), dtype=np.float64)
        jac_inv[_WHEEL_IDX, :] = mat_wheel
        jac_inv[_YAW_IDX, :] = mat_yaw
        return jac_inv
