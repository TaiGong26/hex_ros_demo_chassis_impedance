#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import threading

import numpy as np
import rclpy
import rclpy.node

from geometry_msgs.msg import Twist, Vector3
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboChsCtrl,
    HexRosRoboChsCtrlStamped,
    HexRosRoboChsStateStamped,
    HexRosTeleopKeyboardStateStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseTwist,
    HexDcBaseOdometry,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl,
    HexDcRoboChsState,
    HexDcRoboChsStateStamped,
)
from hex_util_msg.dataclass.dataclass_teleop import HexDcTeleopKeyboardState

from .interface_base import InterfaceBase

_LETTERS = [chr(c) for c in range(ord('a'), ord('z') + 1)]

_CHS_DOF = 8
_ZERO8 = [0.0] * _CHS_DOF


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 500.0)
        self._rate_param["ros"] = self.__node.get_parameter('rate_ros').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### parameters
        self.__node.declare_parameter('rate_teleop', 100.0)
        self.__node.declare_parameter('chs_impedance_kp', [2.0, 2.0])
        self.__node.declare_parameter('chs_impedance_kd', [1.0, 1.0])
        self.__node.declare_parameter('chs_pos_threshold', 0.2)
        self.__node.declare_parameter('chs_yaw_threshold', 0.2)
        self.__node.declare_parameter('chs_type', 'maver_x4')
        self.__node.declare_parameter('chs_bias', 0.02)
        self.__node.declare_parameter('chs_wheel_radius', 0.0625)
        self.__node.declare_parameter('chs_track_width', 0.28)
        self.__node.declare_parameter('chs_wheel_base', 0.424)
        self.__node.declare_parameter('chs_wheel_distance', 0.0)

        self._rate_param.update({
            "teleop":
            self.__node.get_parameter('rate_teleop').value,
        })
        self._impedance_param = {
            "chs_impedance_kp":
            list(self.__node.get_parameter('chs_impedance_kp').value),
            "chs_impedance_kd":
            list(self.__node.get_parameter('chs_impedance_kd').value),
            "chs_pos_threshold":
            self.__node.get_parameter('chs_pos_threshold').value,
            "chs_yaw_threshold":
            self.__node.get_parameter('chs_yaw_threshold').value,
        }
        self._chs_param = {
            "chs_type":
            str(self.__node.get_parameter('chs_type').value),
            "bias":
            float(self.__node.get_parameter('chs_bias').value),
            "wheel_radius":
            float(self.__node.get_parameter('chs_wheel_radius').value),
            "track_width":
            float(self.__node.get_parameter('chs_track_width').value),
            "wheel_base":
            float(self.__node.get_parameter('chs_wheel_base').value),
            "wheel_distance":
            float(self.__node.get_parameter('chs_wheel_distance').value),
        }

        ### publisher
        self.__chs_ctrl_pub = self.__node.create_publisher(
            HexRosRoboChsCtrlStamped,
            'chs_ctrl',
            10,
        )

        ### subscriber
        self.__chs_state_sub = self.__node.create_subscription(
            HexRosRoboChsStateStamped,
            'chs_state',
            self.__chs_state_callback,
            10,
        )
        self.__keyboard_sub = self.__node.create_subscription(
            HexRosTeleopKeyboardStateStamped,
            'teleop_keyboard_state',
            self.__keyboard_callback,
            10,
        )
        self.__chs_state_sub
        self.__keyboard_sub

        ### spin thread
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################
    def pub_chs_ctrl(self, out: HexDcRoboChsCtrl):
        msg = HexRosRoboChsCtrlStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.chs_ctrl = self.__chs_ctrl_to_msg(out)
        self.__chs_ctrl_pub.publish(msg)

    @staticmethod
    def __jnt_to_msg(jnt) -> HexRosJnt:
        return HexRosJnt(
            pos=np.asarray(jnt.pos, dtype=np.float64).tolist(),
            vel=np.asarray(jnt.vel, dtype=np.float64).tolist(),
            eff=np.asarray(jnt.eff, dtype=np.float64).tolist(),
            kp=np.asarray(jnt.kp, dtype=np.float64).tolist(),
            kd=np.asarray(jnt.kd, dtype=np.float64).tolist(),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64).tolist(),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64).tolist(),
        )

    @staticmethod
    def __chs_ctrl_to_msg(chs: HexDcRoboChsCtrl) -> HexRosRoboChsCtrl:
        return HexRosRoboChsCtrl(
            ctrl_mode=int(chs.ctrl_mode),
            jnt=DataInterface.__jnt_to_msg(chs.jnt),
            vel=Twist(
                linear=Vector3(
                    x=chs.vel.linear.x,
                    y=chs.vel.linear.y,
                    z=chs.vel.linear.z,
                ),
                angular=Vector3(
                    x=chs.vel.angular.x,
                    y=chs.vel.angular.y,
                    z=chs.vel.angular.z,
                ),
            ),
        )

    ####################
    ### subscribers
    ####################
    def __chs_state_callback(self, msg: HexRosRoboChsStateStamped):
        self._chs_state_deque.append(self.__chs_state_msg_to_dc(msg))

    def __keyboard_callback(self, msg: HexRosTeleopKeyboardStateStamped):
        self._keyboard_deque.append(self.__keyboard_msg_to_dc(msg))

    @staticmethod
    def __keyboard_msg_to_dc(
            msg: HexRosTeleopKeyboardStateStamped) -> HexDcTeleopKeyboardState:
        kb = msg.keyboard_state
        kwargs = {
            f"key_{letter}": bool(getattr(kb, f"key_{letter}"))
            for letter in _LETTERS
        }
        return HexDcTeleopKeyboardState(**kwargs)

    @staticmethod
    def __jnt_state_to_dc(jnt) -> HexDcBaseJntState:
        return HexDcBaseJntState(
            position=np.asarray(jnt.position, dtype=np.float64),
            velocity=np.asarray(jnt.velocity, dtype=np.float64),
            effort=np.asarray(jnt.effort, dtype=np.float64),
        )

    @staticmethod
    def __odom_to_dc(odom) -> HexDcBaseOdometry:
        pose = odom.pose.pose
        twist = odom.twist.twist
        return HexDcBaseOdometry(
            pose=HexDcBasePose(
                position=HexDcBaseVector3(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                ),
                orientation=HexDcBaseQuaternion(
                    x=pose.orientation.x,
                    y=pose.orientation.y,
                    z=pose.orientation.z,
                    w=pose.orientation.w,
                ),
            ),
            twist=HexDcBaseTwist(
                linear=HexDcBaseVector3(
                    x=twist.linear.x,
                    y=twist.linear.y,
                    z=twist.linear.z,
                ),
                angular=HexDcBaseVector3(
                    x=twist.angular.x,
                    y=twist.angular.y,
                    z=twist.angular.z,
                ),
            ),
        )

    @staticmethod
    def __chs_state_msg_to_dc(
            msg: HexRosRoboChsStateStamped) -> HexDcRoboChsStateStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        chs_state = HexDcRoboChsState(
            jnt=DataInterface.__jnt_state_to_dc(msg.chs_state.jnt),
            odom=DataInterface.__odom_to_dc(msg.chs_state.odom),
        )

        return HexDcRoboChsStateStamped(
            header=header,
            chs_state=chs_state,
        )
