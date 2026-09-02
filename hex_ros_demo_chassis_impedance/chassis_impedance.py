#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import time
import traceback
import threading

import numpy as np
from typing import Optional

from hex_util_ros import quat2yaw
from hex_util_ros.robot_util import angle_norm

from .chs_dyn import MaverX4Dynamics, TriggerADynamics
from .utility import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseTwist,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl,
    HexDcRoboChsCtrlMode,
)

# Joint order: yaw1, wheel1, yaw2, wheel2, yaw3, wheel3, yaw4, wheel4
CHS_DOF = 8


class ChassisImpedance:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("chassis_impedance")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__impedance_param = self.__data_interface.get_impedance_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")

        ### control presets
        self.__impedance_kp = np.asarray(
            self.__impedance_param["impedance_kp"], dtype=np.float64)
        self.__impedance_kd = np.asarray(
            self.__impedance_param["impedance_kd"], dtype=np.float64)
        if self.__impedance_kp.shape != (2, ) or self.__impedance_kd.shape != (
                2, ):
            raise ValueError(
                "impedance_kp and impedance_kd must be [pos, yaw]")
        self.__pos_threshold = float(
            self.__impedance_param["impedance_pos_threshold"])
        self.__yaw_threshold = float(
            self.__impedance_param["impedance_yaw_threshold"])
        self.__force_limit = np.asarray(
            self.__impedance_param["impedance_force_limit"], dtype=np.float64)
        self.__torque_limit = np.asarray(
            self.__impedance_param["impedance_torque_limit"], dtype=np.float64)
        # Desired planar velocity and the current equilibrium pose.
        self.__cmd_vel = np.zeros(3, dtype=np.float64)
        self.__anchor_pose = None

        self.__chs_dyn, self.__chs_dof = self.__create_chs_dyn(
            self.__data_interface.get_chs_param())

        ### threads
        self.__stop_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)

    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()

    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__teleop_thread.start()

    def run(self):
        try:
            self.__work_process()
        except KeyboardInterrupt:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self):
        self.__stop_event.set()
        if self.__teleop_thread.is_alive():
            self.__teleop_thread.join()
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass

    ##############################################################
    # Helpers
    ##############################################################
    @staticmethod
    def __pose_se2_from_odom(odom) -> np.ndarray:
        # hex_util_ros.quat2yaw expects wxyz
        return np.array(
            [
                odom.pose.position.x,
                odom.pose.position.y,
                quat2yaw(
                    np.array(
                        [
                            odom.pose.orientation.w,
                            odom.pose.orientation.x,
                            odom.pose.orientation.y,
                            odom.pose.orientation.z,
                        ],
                        dtype=np.float64,
                    )),
            ],
            dtype=np.float64,
        )

    @staticmethod
    def __twist_body_from_odom(odom) -> np.ndarray:
        return np.array(
            [
                odom.twist.linear.x,
                odom.twist.linear.y,
                odom.twist.angular.z,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def __calc_planar_force(
        total_gain: float,
        err: np.ndarray,
        limit: Optional[float] = None,
    ) -> np.ndarray:
        norm = np.linalg.norm(err)
        if norm <= np.finfo(np.float64).eps:
            return np.zeros(2, dtype=np.float64)
        direction = err / norm
        value = float(total_gain) * np.fabs(err)
        if limit is not None:
            return np.clip(value, 0.0, limit) * direction
        return value * direction

    @staticmethod
    def __calc_torque(
        total_gain: float,
        err: np.ndarray,
        limit: Optional[float] = None,
    ) -> float:
        value = float(total_gain) * err
        if limit is not None:
            return np.clip(value, -limit, limit)
        return value

    def __create_chs_dyn(self, chs_params):
        chs_type = chs_params["chs_type"]
        if chs_type == "maver_x4":
            return MaverX4Dynamics(chs_params), 8
        if chs_type == "trigger_a":
            return TriggerADynamics(chs_params), 3
        raise ValueError(f"Unsupported chs_type: {chs_type}")

    ##############################################################
    # Control builders
    ##############################################################
    def __build_impedance_ctrl(self, eff: np.ndarray) -> HexDcRoboChsCtrl:
        return HexDcRoboChsCtrl(
            ctrl_mode=HexDcRoboChsCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=np.zeros(self.__chs_dof),
                vel=np.zeros(self.__chs_dof),
                eff=np.asarray(eff, dtype=np.float64),
                kp=np.zeros(self.__chs_dof),
                kd=np.zeros(self.__chs_dof),
                lim_vel=np.zeros(self.__chs_dof),
                lim_acc=np.zeros(self.__chs_dof),
            ),
            vel=HexDcBaseTwist(
                linear=HexDcBaseVector3(),
                angular=HexDcBaseVector3(),
            ),
        )

    ##############################################################
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
        while self.__is_running():
            time.sleep(self.__teleop_dt)

            keys = self.__data_interface.get_keyboard_state(latest=True)
            if keys is None:
                continue

            curr_q = bool(keys.key_q)
            if curr_q and not prev_q:
                self.__data_interface.logi(
                    "[chassis_impedance]: stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

    def __work_process(self):
        self.__data_interface.logi(
            "[chassis_impedance]: start impedance control")
        while self.__is_running():
            self.__data_interface.sleep()

            state = self.__data_interface.get_chs_state(latest=True)
            if state is not None:
                cmd_vel = self.__data_interface.get_cmd_vel(latest=True)
                if cmd_vel is not None:
                    self.__cmd_vel = np.array(
                        [
                            cmd_vel.linear.x,
                            cmd_vel.linear.y,
                            cmd_vel.angular.z,
                        ],
                        dtype=np.float64,
                    )

                cur_pose = self.__pose_se2_from_odom(state.chs_state.odom)
                cur_twist = self.__twist_body_from_odom(state.chs_state.odom)

                # The first odometry sample establishes the equilibrium. Once
                # the commanded velocity is large enough, follow the odometry
                # so the moving chassis is no longer pulled toward the old pose.
                if self.__anchor_pose is None:
                    self.__anchor_pose = cur_pose.copy()
                latch_coeff = 100.0 * np.linalg.norm(self.__cmd_vel)
                if latch_coeff > 2.64:  # tanh = 0.99
                    self.__anchor_pose = cur_pose.copy()

                # body-frame SE(2) error toward the equilibrium pose
                err = np.zeros(3)
                c, s = np.cos(cur_pose[2]), np.sin(cur_pose[2])
                err_xy_in_world = self.__anchor_pose[:2] - cur_pose[:2]
                # Rotate the world-frame equilibrium error into the current
                # chassis body frame before applying planar impedance.
                err[:2] = np.clip(
                    np.array(
                        [
                            c * err_xy_in_world[0] + s * err_xy_in_world[1],
                            -s * err_xy_in_world[0] + c * err_xy_in_world[1],
                        ],
                        dtype=np.float64,
                    ),
                    -self.__pos_threshold,
                    self.__pos_threshold,
                )
                err[2] = np.clip(
                    angle_norm(self.__anchor_pose[2] - cur_pose[2]),
                    -self.__yaw_threshold,
                    self.__yaw_threshold,
                )
                stiffness_coeff = 1.0 - np.tanh(latch_coeff)
                damping_coeff = 1.0
                # damping_coeff = 1.0 + np.tanh(latch_coeff)
                force = np.empty(3, dtype=np.float64)
                force[:2] = (self.__calc_planar_force(
                    self.__impedance_kp[0] * stiffness_coeff,
                    err[:2],
                    limit=self.__force_limit[0]) + self.__calc_planar_force(
                        self.__impedance_kd[0] * damping_coeff,
                        self.__cmd_vel[:2] - cur_twist[:2],
                        limit=self.__force_limit[1],
                    ))
                force[2] = (self.__calc_torque(
                    self.__impedance_kp[1] * stiffness_coeff,
                    err[2],
                    limit=self.__torque_limit[0]) + self.__calc_torque(
                        self.__impedance_kd[1] * damping_coeff,
                        self.__cmd_vel[2] - cur_twist[2],
                        limit=self.__torque_limit[1],
                    ))

                # qdot = jac_inv @ twist; calc_jac is pinv(jac_inv), so
                # tau = calc_jac.T @ force from virtual work.
                jnt_pos = np.asarray(state.chs_state.jnt.position,
                                     dtype=np.float64)
                jac = self.__chs_dyn.calc_jac(jnt_pos)
                tau = jac.T @ force
                self.__data_interface.pub_chs_ctrl(
                    self.__build_impedance_ctrl(tau))


def main():
    chassis_impedance = ChassisImpedance()
    try:
        chassis_impedance.start()
        chassis_impedance.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
