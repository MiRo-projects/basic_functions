from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

import sensor_msgs
import std_msgs
import geometry_msgs

# Import system and ROS components
import rospy
import numpy as np
import math
import miro2 as miro
import miro_ros_interface as mri

from miro2.core import pars

import time

class ActionClock(object):

	def __init__(self):

		self.steps_so_far = 0
		self.steps_total = 0
		self.T_norm = 0.0
		self.t_norm = 0.0

	def start(self, steps_total):

		# start integral clock
		self.steps_total = steps_total
		self.steps_so_far = 0

		# start normalised clock
		self.T_norm = 1.0 / steps_total
		self.t_norm = self.T_norm * 0.5

	def toString(self):

		return str(self.steps_so_far) + "/" + str(self.steps_total) + " (" + str(self.t_norm) + ")"

	def advance(self, auto_stop=False):

		# advance
		self.steps_so_far += 1
		self.t_norm += self.T_norm

		# auto stop at complete
		#
		# NB: both integral and norm clocks can overrun the expected time of
		# pattern safely; some patterns wait for another termination condition
		if auto_stop:
			if self.steps_so_far >= self.steps_total:
				self.stop()

	def reset(self):

		# just reset to start, to allow an action to use it as a
		# hold-open button rather than a clock, per se
		self.start(self.steps_total)

	def stop(self):

		# reset clock
		self.steps_total = 0
		self.steps_so_far = 0
		self.T_norm = 0
		self.t_norm = 0

	def isActive(self):

		return self.steps_total > 0

	def linear_profile(self):

		# Returns a profile moving linearly from 0 to 1 (i.e. just normalised time)
		t_norm = self.t_norm
		if t_norm >= 1.0:
			t_norm = 1.0

		return t_norm

	def cosine_profile(self):

		# return a profile moving from 0 to 1 with acceleration
		t_norm = self.t_norm
		if t_norm >= 1.0:
			t_norm = 1.0

		return 0.5 - 0.5 * np.cos(t_norm * np.pi)

	def sine_profile(self):

		# return a profile from -1 to 1, excursing +ve then -ve within cycle (a single period of a sine wave)
		t_norm = self.t_norm
		if t_norm >= 1.0:
			t_norm = 1.0

		return np.sin(t_norm * np.pi * 2.0)

	def cosine_circle_profile(self, phase=0.0):

		# return a profile from 0 to 0 peaking at 1 at mid-point (a single period of a cosine squared)
		t_norm = self.t_norm
		if t_norm >= 1.0:
			t_norm = 1.0

		return 0.5 - 0.5 * np.cos((t_norm * 2.0 + phase) * np.pi)


def deg2rad(n):
    return n * (math.pi / 180)


AZIMUTH = deg2rad(180)
ELEVATION = deg2rad(0)
RANGE = 1

print(AZIMUTH)


# Initialise MiRo client
miro_ros_data = mri.MiroClient()

pars = pars.CorePars()

# create kc object with default (calibration) configuration
# of joints (and zeroed pose of FOOT in WORLD)
kc = miro.utils.kc_interf.kc_miro()

kc_s = miro.utils.kc_interf.kc_miro()
kc_s.setConfig(msg.kinematic_joints.position)



# first, we determine where is the object to which we intend
# to orient, in WORLD frame. this won't change for the duration
# of the action.

# determine desired final gaze target in SENSOR
gaze_target_f_SENSOR = miro.utils.kc_interf.kc_viewline_to_position(
    AZIMUTH,
    ELEVATION,
    RANGE
)

# transform into HEAD
gaze_target_f_HEAD = gaze_target_f_SENSOR

# transform into WORLD
gaze_target_f_WORLD = kc.changeFrameAbs(
    miro.constants.LINK_HEAD,
    miro.constants.LINK_WORLD,
    gaze_target_f_HEAD
)

# get current gaze target at the same range as that, in a form
# that is suitable for use in either HEAD or SENSOR
gaze_target_i_HEAD_OR_SENSOR = miro.utils.kc_interf.kc_viewline_to_position(
    0.0,
    miro.constants.CAM_ELEVATION,
    RANGE
)

# define gaze target in HEAD as canonical gaze target
gaze_target_i_HEAD = gaze_target_i_HEAD_OR_SENSOR

# get current gaze target in WORLD
gaze_target_i_WORLD = kc.changeFrameAbs(
    miro.constants.LINK_HEAD,
    miro.constants.LINK_WORLD,
    gaze_target_i_HEAD
)

# get change in gaze target across movement
dgaze_target_WORLD = gaze_target_f_WORLD - gaze_target_i_WORLD

# decide pattern rate / time
elev = ELEVATION - miro.constants.CAM_ELEVATION
print('elev: ' + str(elev))

rad = np.sqrt(np.square(AZIMUTH) + np.square(elev))
print('rad: ' + str(rad))

sec_ideal = rad * pars.action.orient_speed_sec_per_rad
steps_ideal = int(sec_ideal * pars.timing.tick_hz)
steps = np.clip(steps_ideal, pars.action.orient_min_steps, pars.action.orient_max_steps)

# start action clock
clock2 = ActionClock()
clock2.start(steps)

clock = float(0)
clock_f = float(steps)


# while clock < clock_f:
while clock2.steps_so_far < clock2.steps_total:
    # read clock
    # x = clock / clock_f
    # clock += 1

    # read clock
    x = clock2.cosine_profile()
    print(x)
    clock2.advance(True)

    # compute desired gaze target along a straight line (not quite an arc, but no matter...)
    gaze_target_WORLD_cmd = gaze_target_i_WORLD + x * dgaze_target_WORLD

    # debug how WORLD gaze is intended to, and actually does, change
    gaze_target_WORLD = kc.changeFrameAbs(
        miro.constants.LINK_HEAD,
        miro.constants.LINK_WORLD,
        gaze_target_i_HEAD
    )
    # print gaze_target_WORLD_cmd, gaze_target_WORLD

    # transform into HEAD for actioning as a push
    gaze_x_HEAD = kc.changeFrameAbs(
        miro.constants.LINK_WORLD,
        miro.constants.LINK_HEAD,
        gaze_target_WORLD_cmd
    )

    # we compute a trajectory in WORLD that is a straight line, which
    # in principle will cause us to shimmy backwards rather than simply
    # turning our head. in practice, this is prevented by using the flag
    # PUSH_FLAG_IMPULSE anyway, but for belt and braces we reconstruct
    # the arc, here, approximately, unless flagged off.
    gaze_x_HEAD *= np.linalg.norm(gaze_target_f_SENSOR) / np.linalg.norm(gaze_x_HEAD)

    # prepare push
    push = miro.utils.kc.KinematicPush()
    push.link = miro.constants.LINK_HEAD
    push.flags = 0 | miro.constants.PUSH_FLAG_IMPULSE | miro.constants.PUSH_FLAG_NO_TRANSLATION
    push.pos = gaze_target_i_HEAD
    push.vec = gaze_x_HEAD - gaze_target_i_HEAD

    # apply push to local kc
    kc.push(push)

    # get config & dpose from kc
    config = kc.getConfig()
    dpose = kc.getPoseChange() * miro.constants.PLATFORM_TICK_HZ

    miro_ros_data.pub_kinematic(
        config[0],
        config[1],
        config[2],
        config[3],
    )

    (whl_l, whl_r) = miro.utils.cmd_vel2wheel_speed(dpose[0], dpose[1])

    miro_ros_data.pub_velocity(whl_l, whl_r)

    time.sleep(0.02)









    # # # #

# # this example illustrates how to use the Kinematic Chain
# # tool "kc" to transform positions and directions between
# # the different frames of reference of the robot. the frames
# # are listed in miro_constants.py.
#
# # create kc object with default (calibration) configuration
# # of joints (and zeroed pose of FOOT in WORLD)
# kc = miro.utils.kc_interf.kc_miro()
#
# # create objects in HEAD
# pos = miro.utils.get("LOC_NOSE_TIP_HEAD")
# vec = np.array([-1.0, 0.0, 0.0])
#
# while True:
#     # transform to WORLD (note use of "Abs" and "Rel"
#     # for positions and directions, respectively)
#     posw = kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pos)
#     vecw = kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, vec)
#
#     # report
#     print('Pose: ' + str(pos))
#     print(' & Vector: ' + str(vec) + '\n')
#
#     # print('Pose (W): ' + str(posw))
#     # print(' & Vector (W): ' + str(vecw) + '\n')
#     # print posw, vecw
#
#     # update configuration based on data (imagine this came
#     # from /miro/sensors/kinematic_joints)
#     #
#     # NB: the immobile joint "TILT" is always at the same
#     # angle, "TILT_RAD_CALIB"
#     kinematic_joints = np.array([
#         miro.constants.TILT_RAD_CALIB,
#         miro_ros_data.sensors_kinematic_joints['lift'],
#         miro_ros_data.sensors_kinematic_joints['yaw'],
#         miro_ros_data.sensors_kinematic_joints['pitch']
#     ])
#     # kinematic_joints = np.array([miro.constants.TILT_RAD_CALIB, np.radians(30.0), np.radians(15.0), np.radians(0.0)])
#     kc.setConfig(kinematic_joints)
#
#     # transform to WORLD
#     posw = kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, pos)
#     vecw = kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, vec)
#
#     config = kc.getConfig()
#     dpose = kc.getPoseChange() * miro.constants.PLATFORM_TICK_HZ
#
#     print(config)
#     print(dpose)
#
#     # report
#     print('Pose (W): ' + str(posw))
#     print(' & Vector (W): ' + str(vecw) + '\n')
#     # print posw, vecw
#     # miro_ros_data.pub_kinematic(
#     #     kinematic_joints[0],
#     #     kinematic_joints[1],
#     #     kinematic_joints[2],
#     #     kinematic_joints[3],
#     # )
#
#     kc.push()
