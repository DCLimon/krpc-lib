"""
Classes to allow throttle control based on different parameters.

Classes
-------

ThrottleControl
    Contains implementation methods to control throttle based on input
    from subclasses.

MaxQThrottler
    Takes a dynamic pressure (q) at max-q and passes to ThrottleControl.

SpeedThrottle
    Takes a user-defined max ground speed and passes to ThrottleControl.
"""
# 3rd party packs
import krpc

# local packs
from base import *


class ThrottleControl(Craft):

    _mv = 0  # throttle output; class variable to clash b/w instances

    def __init__(self, set_point):
        super().__init__()

        # Define control parameters to be overridden by PV-specific
        # throttle
        self._pv = None  # locked to data stream when used by subclasses
        # PV = process variable, value ultimately being controlled,
        # e.g. vessel speed
        self._sp = set_point
        # SP = set point, the ideal value of PV

        self._lower_bound = 0.9*self._sp
        self._upper_bound = 1.1*self._sp

    def __call__(self):
        ThrottleControl._mv = self._throttle_val()
        self.throttle = ThrottleControl._mv

    def _throttle_val(self):
        self._check_pv()

        if self._pv < self._lower_bound:
            return 1.0
        elif self._pv > self._upper_bound:
            return 0.0
        else:
            return Craft.min_max_norm(self._pv, self._lower_bound,
                                      self._upper_bound)

    def _check_pv(self):
        if not self._pv:
            raise TypeError('PV=None because it was not redefined when'
                            'defining this instance.')


class MaxQThrottler(ThrottleControl):
    def __init__(self, max_q):
        set_point = max_q  # max_q is subclass-specific set_point
        super(MaxQThrottler, self).__init__(set_point)

        self._pv = self._q  # self._q defined defined in base.py
        self.max_q = max_q
        self._sp = self.max_q


class SpeedThrottler(ThrottleControl):
    def __init__(self, max_speed):
        set_point = max_speed  # max_speed = subclass-specific set_point
        super(SpeedThrottler, self).__init__(set_point)

        self._pv = self._ground_speed  # self._ground_speed from base.py
        self.max_speed = max_speed
        self._sp = self.max_speed
