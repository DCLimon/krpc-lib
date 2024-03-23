"""
Implementation of Euler angles (pitch, yaw, roll)
"""

# 3rd party packs
import numpy as np
import krpc

# local packs
from base import *

class EulerAngles(Craft):
    _pitch_axis = i_HAT
    _yaw_axis = j_HAT
    _roll_axis = k_HAT

    def __init__(self, xyz_vector=None, ref_frame=None):
        super().__init__()

        self.xyz_vec = xyz_vector / np.linalg.norm(xyz_vector)
        self.facing_hat = np.array((self._facing_x,
                                    self. _facing_y,
                                    self._facing_z))

    @property
    def _pitch_radians(self):
        result = Craft.angle_between_vectors(self.xyz_vec,
                                             np.array(1., 0., 1.))
        if result > np.pi / 2:
            return np.pi / 2
        elif result < -np.pi / 2:
            return -np.pi / 2
        else:
            return result

    @property
    def _yaw_radians(self):
        result = Craft.angle_between_vectors(self.xyz_vec,
                                             np.array(1., 1., 0.))
        if result > np.pi:
            return result - (2*np.pi)
        elif result < -np.pi:
            return result + (2*np.pi)
        else:
            return result

    # @property
    # def _roll_radians(self):
        # use transform matrix

    @property
    def _facing_x(self):
        # scale x such that vertical flight has only roll
        return np.cos(self._yaw_radians) * np.cos(self._pitch_radians)

    @property
    def _facing_y(self):
        return np.sin(self._pitch_radians)


    @property
    def _facing_z(self):
        # scale z such that vertical flight has only roll
        return np.sin(self._yaw_radians) * np.cos(self._pitch_radians)


# find p(q) where p is a sigmoid fxn where p(-inf)=p_min, p(+inf)=p_max,
#   p(0)=p_mid, dp/dq depends on logistic growth factor k, and q_i is an
#   input value to be normalized b/w p_min & p_max
def logistic_norm(q_i, p_min=0, p_max=1, logistic_growth=1):
    p_mid = (p_max - p_min) / 2  # norm midpoint
    k = logistic_growth
    return p_max / (1 + math.e ** (-k * (q_i - p_mid)))


# given a_i within range a_min to a_max, find b_i = a_i when range a_min
#   to a_max is normalized to range b_min to b_max
# can be used to contract, e.g. find pitch input (-1 to 1) given pitch
#   error (0 to 90 deg)
# can be used to expand, e.g. find target pitch angle (90 to 0) given
#   current altitude (0 to turn_end_altitude)
def min_max_norm(a_i, a_min=None, a_max=None, b0=0, b1=1):
    # alter to accept lists
    if a_min is None or a_max is None:
        print('Normalized with an analogous logistic function in'
              'order to handle infinite input bound(s)')
        return logistic_norm(a_i, b0, b1)
    return ((b1-b0)*(a_i-a_min) / (a_max-a_min)) + b0


# find closest approach of object travelling from a0 in the a-hat
#   direction to a static point p
def closest_approach_posn(point_vec, a_dir_vec, a_start_posn=(0, 0, 0)):
    a_hat = (a_dir_vec - a_start_posn) / np.linalg.norm(a_dir_vec -
                                                        a_start_posn)
    b = np.array(point_vec) - np.array(a_start_posn)
    return np.dot(a_hat, b) * a_hat

