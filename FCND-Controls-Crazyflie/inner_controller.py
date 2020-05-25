"""Controller class for computing attitude and thrust commands.

implementation of the inner loop controller which controls velocity through attitude and thrust commands.

@author Adrien Perkins
"""

import numpy as np

# some necessary constants
DRONE_M = 0.031         # [kg]
GRAVITY_MAG = 9.81      # [m/s^2] -> magnitude only
MAX_THRUST_N = 0.63     # the maximum amount of thrust the crazyflie can generate in [N] - DO NOT EDIT


class InnerLoopController(object):

    def __init__(self):

        # the gains that are needed
        self._kp_vel = 0.0   # the gain on the velocity to get attitude
        self._kp_hdot = 0.0  # the gain on the vertical velocity to get accel

        # some limits to use
        self._bank_max = np.radians(20)     # max bank (roll and pitch) angle - in radians
        self._haccel_max = 1.2              # the maximum vertical acceleration in [m/s^2]

    def velocity_control(self, vel_cmd, vel):
        """compute attitude and thrust commands to achieve a commanded velocity vector.

        Use a PID controller (or your controller of choice) to compute the attitude (roll/pitch) to
        achieve the commanded (North, East) velocity and compute a normalized thrust to achieve the
        commanded down velocity.

        Args:
            vel_cmd: the commanded velocity vector as a numpy array [Vnorth, Veast, Vdown] in [m/s]
            vel: the current velocity vector as a numpy array [Vnorth, Veast, Vdown] in [m/s]
        """

        # change down velocities to up hdots
        hdot_cmd = -vel_cmd[2]
        hdot = -vel[2]

        # Student TODO: compute an attitude command from the given velocity command
        pitch = -self._kp_vel * (vel_cmd[0] - vel[0])  # note the sign change!  Remember + pitch is up, meaning it will send out drone backwards!
        roll = self._kp_vel * (vel_cmd[1] - vel[0])

        # add some limits
        pitch_cmd = np.clip(pitch, -self._bank_max, self._bank_max)
        roll_cmd = np.clip(roll, -self._bank_max, self._bank_max)
        
        # Student TODO: compute a normalized thrust from the given hdot command
        accel_cmd = self._kp_hdot * (hdot_cmd - hdot)  # compute acceleration from vertical velocity error
        accel_cmd = np.clip(accel_cmd, -self._haccel_max, self._haccel_max)  # saturate as desired
        thrust_cmd_N = DRONE_M * (accel_cmd + GRAVITY_MAG) / (np.cos(pitch_cmd) * np.cos(roll_cmd))  # compute thrust in N positive up

        # need to normalize the thrust
        thrust_cmd = thrust_cmd_N / MAX_THRUST_N

        # return a tuple with the roll, pitch, and thrust commands
        return (roll_cmd, pitch_cmd, thrust_cmd)
