"""
A collection of basic variables and methods for subclass & module use.

Constants
---------

IDENT
    3x3 Numpy identity matrix made of component x, y, z unit vectors
i-HAT
    x-direction unit vector (1, 0, 0)
j-HAT
    y-direction unit vector (0, 1, 0)
k-HAT
    z-direction unit vector (0, 0, 1)

Classes
-------

Craft
    Contains general variables and methods that should be accessible to
    subclasses.

    Subclasses:
        in throttle.py
            - ThrottleController
            - MaxQThrottler
            - SpeedThrottler
        in attitude.py
            - RollController
            - PitchController
            - RotationRateController
            - AttitudeController
"""

import krpc
import math
import numpy as np

# constants
IDENT = np.identity
i_HAT = IDENT[0, :]
j_HAT = IDENT[1, :]
k_HAT = IDENT[2, :]

pi = math.pi


class Craft:

    def __init__(self):
        # active vessel variables
        self.conn = krpc.connect(name='')
        self.KSC = self.conn.space_center
        self.vessel = self.KSC.active_vessel
        self.ut = self.conn.add_stream(
            getattr, self.conn.space_center, "ut")

        # control variables
        self.control = self.vessel.control
        self.throttle = self.control.throttle
        self.autopilot = self.vessel.auto_pilot

        # body reference frames
        self.srf_frame = self.conn.add_stream(
            getattr, self.body, 'reference_frame')
        self.obt_frame = self.conn.add_stream(
            getattr, self.body, 'non_rotating_reference_frame')

        # vessel reference frames; see var_ref.rst for clarification PRN
        self.vessel.frame = self.conn.add_stream(
            getattr, self.vessel, 'reference_frame')
        self.vessel.obt_frame = self.conn.add_stream(
            getattr, self.vessel, 'orbital_reference_frame')
        #   note this is different than self.body.obt_frame
        self.vessel.srf_frame = self.conn.add_stream(
            getattr, self.vessel, 'surface_reference_frame')
        #   note this is different than self.body.srf_frame
        self.vessel.srf_vel_frame = self.conn.add_stream(
            getattr, self.vessel, 'surface_velocity_reference_frame')

        # flight dynamics
        self._alt_msl = self.conn.add_stream(
            getattr, self.flight(self.srf_frame), 'mean_altitude')
        self._ground_speed = self.conn.add_stream(
            getattr, self.flight(self.srf_frame), 'speed')
        self._q = self.conn.add_stream(
            getattr, self.flight(self.srf_frame), 'dynamic_pressure')

        # orbital dynamics
        self.obt = self.vessel.orbit
        self.body = self.obt.body

    def flight(self, vessel_ref_frame):
        return self.vessel.flight(vessel_ref_frame)

    @staticmethod
    def angle_between_vectors(np_1x3_1, np_1x3_2,
                              angle_in_radians=True):
        a = np_1x3_1
        a_mag = np.linalg.norm(a)
        b = np_1x3_2
        b_mag = np.linalg.norm(b)
        theta_rad = np.arccos(np.dot(a, b) / (a_mag * b_mag))
        theta_deg = theta_rad * (180. / np.pi)
        if angle_in_radians:
            return float(theta_rad)
        return float(theta_deg)

    # scales np_1x3 so that it has the same direction but an overall
    # magnitude of 1
    @staticmethod
    def as_unit_vector(np_1x3):
        return np_1x3 / np.linalg.norm(np_1x3)

    # given unbounded input_val, return output value that is equal to
    # input_val bounded to values between min_val and max_val
    @staticmethod
    def clamp(input_val, min_val, max_val):
        return max(min_val, min(input_val, max_val))

    @staticmethod
    def logistic_norm(unnorm_val, output_lowbound=0, output_hibound=1,
                      logistic_growth=1):
        x = unnorm_val
        x_mid = (output_hibound - output_lowbound) / 2  # norm midpoint
        k = logistic_growth
        return output_hibound / (1 + math.e ** (-k * (x - x_mid)))

    @staticmethod
    def min_max_norm(unnorm_val, unnorm_min=None, unnorm_max=None,
                     output_lowbound=0, output_hibound=1):
        # alter to accept lists
        if unnorm_min is None or unnorm_max is None:
            print('Normalized with an analogous logistic function in'
                  'order to handle infinite input bound(s)')
            return Craft.logistic_norm(unnorm_val,
                                       output_lowbound,
                                       output_hibound)
        return (((output_hibound - output_lowbound)
                 * ((unnorm_val - unnorm_min)
                    / (unnorm_max - unnorm_min)))
                + output_lowbound)

        # self.camera = self.KSC.camera
        # self.CameraMode = self.KSC.CameraMode
        #

        #
        #
        # self.rotational_period = self.body.rotational_period
        # self.mu = self.body.gravitational_parameter
        # self.radius_eq = self.body.equatorial_radius
        #
        #
        # self.parts = self.vessel.parts
        # self.engines = self.parts.with_module("ModuleEnginesRF")
        #
        # self.thrust = self.conn.add_stream(getattr, self.vessel, 'thrust')
        # self.max_thrust = self.conn.add_stream(getattr, self.vessel, 'max_thrust')
        # self.specific_impulse = self.conn.add_stream(getattr, self.vessel, 'vacuum_specific_impulse')
        # self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        # self.apoapsis_altitude = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        # self.periapsis_altitude = self.conn.add_stream(getattr, self.vessel.orbit, 'periapsis_altitude')
        # self.apoapsis_radius = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis')
        # self.periapsis_radius = self.conn.add_stream(getattr, self.vessel.orbit, 'periapsis')
        # self.mean_anomaly = self.conn.add_stream(getattr, self.vessel.orbit, 'mean_anomaly')
