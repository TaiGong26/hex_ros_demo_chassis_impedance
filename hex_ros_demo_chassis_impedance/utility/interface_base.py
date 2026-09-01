#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTwist
from hex_util_msg.dataclass.dataclass_robo import HexDcRoboChsCtrl
from hex_util_msg.dataclass.dataclass_robo import HexDcRoboChsStateStamped
from hex_util_msg.dataclass.dataclass_teleop import HexDcTeleopKeyboardState


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._impedance_param = {}
        self._chs_param = {}

        ### rx msg queues
        self._chs_state_deque = deque(maxlen=100)
        self._keyboard_deque = deque(maxlen=100)
        self._cmd_vel_deque = deque(maxlen=100)

        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_impedance_param(self) -> dict:
        return self._impedance_param

    def get_chs_param(self) -> dict:
        return self._chs_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_chs_ctrl(self, out: HexDcRoboChsCtrl):
        raise NotImplementedError("InterfaceBase.pub_chs_ctrl")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # chassis state
    def get_chs_state(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboChsStateStamped]:
        return self.deque_helper(self._chs_state_deque, latest)

    # keyboard state
    def get_keyboard_state(
        self,
        latest: bool = False,
    ) -> Optional[HexDcTeleopKeyboardState]:
        return self.deque_helper(self._keyboard_deque, latest)

    # cmd_vel (desired chassis velocity)
    def get_cmd_vel(self, latest: bool = False) -> Optional[HexDcBaseTwist]:
        return self.deque_helper(self._cmd_vel_deque, latest)
