#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import numpy as np
import rospy

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
        rospy.init_node(name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 500.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### parameters
        self._rate_param.update({
            "teleop": rospy.get_param('~rate_teleop', 100.0),
        })
        self._model_param = {
            "urdf":
            rospy.get_param('~model_urdf', ""),
            "frame_id":
            rospy.get_param('~model_frame_id', "base_link"),
        }
        self._impedance_param = {
            "chs_impedance_kp":
            list(rospy.get_param('~chs_impedance_kp',
                                 rospy.get_param('~impedance_kp', [0.5, 0.5]))),
            "chs_impedance_kd":
            list(rospy.get_param('~chs_impedance_kd',
                                 rospy.get_param('~impedance_kd', [0.0, 0.0]))),
            "chs_stable_pos":
            list(rospy.get_param('~chs_stable_pos', list(_ZERO8))),
            "chs_stable_vel":
            list(rospy.get_param('~chs_stable_vel', list(_ZERO8))),
            "chs_kp":
            list(
                rospy.get_param(
                    '~chs_kp', [20.0, 0.0, 20.0, 0.0, 20.0, 0.0, 20.0, 0.0])),
            "chs_kd":
            list(rospy.get_param('~chs_kd', [1.0] * _CHS_DOF)),
            "chs_pos_threshold":
            rospy.get_param('~chs_pos_threshold', 0.2),
            "chs_yaw_threshold":
            rospy.get_param('~chs_yaw_threshold', 0.2),
            "chs_vel_kd":
            list(
                rospy.get_param(
                    '~chs_vel_kd',
                    [0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0])),
            "arrive_threshold":
            rospy.get_param('~arrive_threshold', 0.06),
        }
        self._chs_param = {
            "chs_type":
            str(rospy.get_param('~chs_type', 'maver_x4')),
            "bias":
            float(rospy.get_param('~chs_bias', 0.02)),
            "wheel_radius":
            float(rospy.get_param('~chs_wheel_radius', 0.0625)),
            "track_width":
            float(rospy.get_param('~chs_track_width', 0.28)),
            "wheel_base":
            float(rospy.get_param('~chs_wheel_base', 0.424)),
            "wheel_distance":
            float(rospy.get_param('~chs_wheel_distance', 0.0)),
        }

        ### publisher
        self.__chs_ctrl_pub = rospy.Publisher(
            'chs_ctrl',
            HexRosRoboChsCtrlStamped,
            queue_size=10,
        )

        ### subscriber
        self.__chs_state_sub = rospy.Subscriber(
            'chs_state',
            HexRosRoboChsStateStamped,
            self.__chs_state_callback,
        )
        self.__keyboard_sub = rospy.Subscriber(
            'teleop_keyboard_state',
            HexRosTeleopKeyboardStateStamped,
            self.__keyboard_callback,
        )
        self.__cmd_vel_sub = rospy.Subscriber(
            'cmd_vel',
            Twist,
            self.__cmd_vel_callback,
        )
        self.__chs_state_sub
        self.__keyboard_sub
        self.__cmd_vel_sub

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    ####################
    ### publishers
    ####################
    def pub_chs_ctrl(self, out: HexDcRoboChsCtrl):
        msg = HexRosRoboChsCtrlStamped()
        msg.header.stamp = rospy.Time.now()
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

    def __cmd_vel_callback(self, msg: Twist):
        self._cmd_vel_deque.append(self.__twist_msg_to_dc(msg))

    @staticmethod
    def __twist_msg_to_dc(msg: Twist) -> HexDcBaseTwist:
        return HexDcBaseTwist(
            linear=HexDcBaseVector3(
                x=float(msg.linear.x),
                y=float(msg.linear.y),
                z=float(msg.linear.z),
            ),
            angular=HexDcBaseVector3(
                x=float(msg.angular.x),
                y=float(msg.angular.y),
                z=float(msg.angular.z),
            ),
        )

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
                secs=int(msg.header.stamp.secs),
                nsecs=int(msg.header.stamp.nsecs),
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
