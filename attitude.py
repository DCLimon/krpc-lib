"""
Several related classes working together to control vessel attitude.

Classes
-------

RollController
    Simple proportional controller to control vessel's roll axis.

PitchController
    Simple proportional controller to control vessel's pitch axis.

RotationRateController
    PID (simple-PID) control of pitch, yaw, roll based on target vector.

AttitudeController

"""

# standard packages
import math

# 3rd party packages
import krpc
import numpy as np
import simple_pid

# local packages
from base import *


class RollController(Craft):

    def __init__(self, target_roll, ref_frame):
        super().__init__()

        self.target_roll = target_roll * (
                math.pi / 180.)  # roll in rads
        self._roll = (math.pi / 180) * self.conn.add_stream(
            getattr, self.flight(ref_frame), 'roll')
        #   flight(ref).roll returns roll angle from -180 to +180
        #   relative to plane of horizon

        # often use self.vessel.surface_reference_frame in atmo,
        # where (x,y,z) directions:
        # x = from body center -> thru vessel CoG -> directly
        #     upwards from CoG (body's radial(+) line thru CoG)
        # y = North tangent to surface; where a compass laying on
        #     the ground below the vessel would point
        # z = East tangent to surface; 90 deg. CW from compass in y

    def __call__(self):
        roll = self._roll_error
        self.control.roll = roll

    @property
    def _roll_error(self):
        return self.target_roll - self._roll


class PitchController(Craft):

    def __init__(self, target_pitch_deg, ref_frame=None):
        super().__init__()

        if ref_frame is None:
            ref_frame = self.vessel.srf_frame

        self.target_pitch = target_pitch_deg * (math.pi / 180.)
        self._velocity_vector = self.conn.add_stream(
            getattr, self.flight(ref_frame), 'velocity')
        #   return (x,y,z) of vessel velocity direction, must find
        # magnitude if wanted to know speed

    def __call__(self):
        norm_pitch = Craft.min_max_norm(self._pitch_error,
                                        -math.pi, math.pi, -1, 1)
        self.control.pitch = float(norm_pitch)

    # @property
    # def _velocity_vector_pitch(self):
    #     horizon_plane = j_HAT + k_HAT
    #     return Craft.angle_between_vectors(
    #         np.array(self._velocity_vector), horizon_plane)

    # quaternion witchcraft
    @property
    def _pitch_error(self):
        pitch_v_vect = np.dot(self._velocity_vector, i_HAT)
        return (self.target_pitch/2) - pitch_v_vect


class RotationRateController(Craft):

    def __init__(self, ref_frame, target_pitch=0, target_yaw=0,
                 target_roll=0):
        super().__init__()

        self.target = np.array((target_pitch, target_yaw, target_roll))
        self._ang_velocity_vector = self.conn.add_stream(
            self.vessel.angular_velocity(ref_frame)
        )

        # Define pitch, yaw, roll axes within chosen ref frame
        self._pitch_dir = self.conn.add_stream(
            self.KSC.transform_direction, i_HAT,
            self.vessel.reference_frame, ref_frame
        )
        self._yaw_dir = self.conn.add_stream(
            self.KSC.transform_direction, k_HAT,
            self.vessel.reference_frame, ref_frame
        )
        self._roll_dir = self.conn.add_stream(self.vessel.direction,
                                              ref_frame)

        # create PID controllers with default parameters; can change
        # tuning parameters here
        self.pitch_pid = simple_pid.PID(Kp=1.0, Ki=0.0, Kd=0.0,
                                        setpoint=0,
                                        sample_time=0.01,
                                        output_limits=(-1, 1),
                                        auto_mode=True,
                                        proportional_on_measurement=(
                                            False))
        self.yaw_pid = simple_pid.PID(Kp=1.0, Ki=0.0, Kd=0.0,
                                      setpoint=0,
                                      sample_time=0.01,
                                      output_limits=(-1, 1),
                                      auto_mode=True,
                                      proportional_on_measurement=(
                                          False))
        self.roll_pid = simple_pid.PID(Kp=1.0, Ki=0.0, Kd=0.0,
                                       setpoint=0,
                                       sample_time=0.01,
                                       output_limits=(-1, 1),
                                       auto_mode=True,
                                       proportional_on_measurement=(
                                           False))

    def __call__(self):
        self.control.pitch = self.pitch_pid(self._attitude_error[0])
        self.control.yaw = self.yaw_pid(self._attitude_error[1])
        self.control.roll = self.roll_pid(self._attitude_error[2])

    # Calculate an error value and scale by the determined ref frame
    # transformation factors
    @property
    def _attitude_error(self):
        err = self.target - self._ang_velocity_vector
        pitch = np.dot(err, self._pitch_dir())
        yaw = np.dot(err, self._yaw_dir())
        roll = np.dot(err, self._roll_dir())
        return np.array((pitch, yaw, roll))


class AttitudeController(Craft):
    def __init__(self, target_direction_vector, target_roll, ref_frame):
        super().__init__()

        # property setter returns direction of target attitude
        self.target_direction = target_direction_vector  # unit vector
        self.target_roll = target_roll

        # returns unit vector
        self._direction = self.conn.add_stream(self.vessel.direction,
                                               ref_frame)
        self._roll = self.conn.add_stream(getattr,
                                          self.flight(ref_frame),
                                          'roll')
        self.rate_controller = RotationRateController(ref_frame,
                                                      target_pitch=0,
                                                      target_yaw=0,
                                                      target_roll=0)
        self._pitch_pid = self.rate_controller.pitch_pid
        self._yaw_pid = self.rate_controller.yaw_pid
        self._roll_pid = self.rate_controller.roll_pid

    # quaternion witchcraft
    def __call__(self):
        self.rate_controller.target = np.cross(self.target_direction,
                                               np.array(
                                                   self._direction()))
        if self.target_roll is not None:
            self.rate_controller.target += self.target_direction * (
                        self.roll_error / 90)
        self.rate_controller()

    @property
    def target_direction(self):
        return self._target_direction

    @target_direction.setter
    def target_direction(self, direction):
        dir_ = np.array(direction)
        self._target_direction = Craft.as_unit_vector(dir_)

    @property
    def target_roll(self):
        return self._target_roll

    @target_roll.setter
    def target_roll(self, roll):
        self._target_roll = roll

    @property
    def error(self):
        return self.target_direction - np.array(self._direction())

    @property
    def roll_error(self):
        if self.target_roll is None:
            return 0
        return self.target_roll - self._roll()
