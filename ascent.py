"""
Classes for ascent to orbit.

Classes
-------
Ascent
    Controls craft ascent to a specified target altitude with zero
    inclination
"""
# standard packs
import math
import time

# 3rd party packs
import krpc

# local packs
from base import *
from attitude import *

class Ascent(Craft):

    def __init__(self, target_altitude, turn_start_altitude=250,
                 turn_start_speed=100):
        super().__init__()

        self.target_alt = target_altitude
        self.turn_alt = turn_start_altitude
        self.turn_speed = turn_start_speed
        self._turn_end_alt = self.body.atmosphere_depth * 0.75
        self._max_q = 7000

        # tele streams
        self.ut
