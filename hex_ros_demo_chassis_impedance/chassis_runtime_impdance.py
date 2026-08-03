#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-03
################################################################

import os
import sys
import time
import traceback
import threading

import numpy as np

from hex_util_ros import quat2yaw
from hex_util_ros.robot_util import angle_norm

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface

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
YAW_IDX = [0, 2, 4, 6]


class ChassisRuntimeImpdance:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("chassis_runtime_impdance")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__impedance_param = self.__data_interface.get_impedance_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")

        ### control presets
        self.__impedance_kp = np.asarray(
            self.__impedance_param["chs_impedance_kp"], dtype=np.float64)
        self.__impedance_kd = np.asarray(
            self.__impedance_param["chs_impedance_kd"], dtype=np.float64)
        self.__chs_pos_threshold = float(
            self.__impedance_param["chs_pos_threshold"])
        self.__chs_yaw_threshold = float(
            self.__impedance_param["chs_yaw_threshold"])
        self.__start_pose = np.zeros(3, dtype=np.float64)

        # Maver X4 chassis geometry (from params) + derived terms
        self.__chs_params = dict(self.__data_interface.get_chs_param())
        self.__chs_params["bias_inv"] = 1.0 / self.__chs_params["bias"]
        self.__chs_params["wheel_radius_inv"] = (
            1.0 / self.__chs_params["wheel_radius"])
        # distance from base origin to each yaw joint
        self.__chs_params["wheel_distance"] = 0.5 * np.ones(
            4) * np.sqrt(self.__chs_params["track_width"]**2 +
                         self.__chs_params["wheel_base"]**2)
        temp_beta = np.arctan2(self.__chs_params["track_width"],
                               self.__chs_params["wheel_base"])
        self.__chs_params["beta"] = np.array(
            [temp_beta, np.pi - temp_beta, temp_beta - np.pi, -temp_beta])

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

    def __calc_jac_inv(self, yaw: np.ndarray) -> np.ndarray:
        """
        Inverse Jacobian (8x3): motor_vel = jac_inv @ [vx, vy, omega].
        Joint order: [yaw1, wheel1, yaw2, wheel2, yaw3, wheel3, yaw4, wheel4].
        Same construction as hex_ros_sim_maver_x4.MujocoSim.__calc_jac_inv.
        """
        yaw = np.asarray(yaw, dtype=np.float64).reshape(4)
        sin_theta = np.sin(yaw)
        cos_theta = np.cos(yaw)
        sin_theta_beta = np.sin(yaw - self.__chs_params["beta"])
        cos_theta_beta = np.cos(yaw - self.__chs_params["beta"])

        mat_wheel = np.column_stack(
            (cos_theta, sin_theta, self.__chs_params["wheel_distance"] *
             sin_theta_beta)) * self.__chs_params["wheel_radius_inv"]
        mat_yaw = np.column_stack(
            (-sin_theta, cos_theta,
             self.__chs_params["wheel_distance"] * cos_theta_beta -
             self.__chs_params["bias"])) * self.__chs_params["bias_inv"]

        jac_inv = np.empty((CHS_DOF, 3), dtype=np.float64)
        jac_inv[0::2, :] = mat_yaw
        jac_inv[1::2, :] = mat_wheel
        return jac_inv

    ##############################################################
    # Control builders
    ##############################################################
    def __build_impedance_ctrl(self, eff: np.ndarray) -> HexDcRoboChsCtrl:
        return HexDcRoboChsCtrl(
            ctrl_mode=HexDcRoboChsCtrlMode.MIT,
            jnt=HexDcBaseJntFull(
                pos=np.zeros(CHS_DOF),
                vel=np.zeros(CHS_DOF),
                eff=np.asarray(eff, dtype=np.float64),
                kp=np.zeros(CHS_DOF),
                kd=np.zeros(CHS_DOF),
                lim_vel=np.zeros(CHS_DOF),
                lim_acc=np.zeros(CHS_DOF),
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
                    "[chassis_runtime_impdance]: stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

    def __work_process(self):
        self.__data_interface.logi(
            "[chassis_runtime_impdance]: start impedance control")
        while self.__is_running():
            self.__data_interface.sleep()

            state = self.__data_interface.get_chs_state(latest=True)
            if state is not None:
                cur_pose = self.__pose_se2_from_odom(state.chs_state.odom)
                cur_twist = self.__twist_body_from_odom(state.chs_state.odom)

                # body-frame SE(2) error toward the equilibrium pose
                err = np.zeros(3)
                c, s = np.cos(cur_pose[2]), np.sin(cur_pose[2])
                err_xy_in_world = self.__start_pose[:2] - cur_pose[:2]
                err[:2] = np.clip(
                    np.array(
                        [
                            c * err_xy_in_world[0] + s * err_xy_in_world[1],
                            -s * err_xy_in_world[0] + c * err_xy_in_world[1],
                        ],
                        dtype=np.float64,
                    ),
                    -self.__chs_pos_threshold,
                    self.__chs_pos_threshold,
                )
                err[2] = np.clip(
                    angle_norm(self.__start_pose[2] - cur_pose[2]),
                    -self.__chs_yaw_threshold,
                    self.__chs_yaw_threshold,
                )
                force = self.__impedance_kp * err - self.__impedance_kd * cur_twist

                # qdot = jac_inv @ twist, virtual work => F = jac_inv.T @ tau
                # so tau = (jac_inv.T)^+ @ F
                jnt_pos = np.asarray(state.chs_state.jnt.position,
                                     dtype=np.float64)
                jac_inv = self.__calc_jac_inv(jnt_pos[YAW_IDX])
                tau = np.linalg.pinv(jac_inv.T) @ force
                self.__data_interface.pub_chs_ctrl(
                    self.__build_impedance_ctrl(tau))


def main():
    chassis_runtime_impdance = ChassisRuntimeImpdance()
    try:
        chassis_runtime_impdance.start()
        chassis_runtime_impdance.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
