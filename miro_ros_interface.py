# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage
from miro2_msg import msg

# MiRo-E parameters
import constants as con

# Image handling
import cv2
from PIL import Image as Im
from PIL import ImageOps

# Other imports
import os
import rospy
import numpy as np
import miro2 as miro


class MiRo:
	def __init__(self, name):
		# Check node name validity and prefix with underscore
		if name:
			name = '_' + filter(str.isalnum, str(name))

		# Initialise ROS node ('disable_rostime=True' needed to work in PyCharm)
		rospy.init_node('MRI' + name, anonymous=True, disable_rostime="PYCHARM_HOSTED" in os.environ)

		self.tr = '/' + os.getenv('MIRO_ROBOT_NAME') + '/'  # ROS topic root
		self.qs = 2                                         # Publisher queue size
		self.sleep = 0.5                                    # Initialisation sleep duration


class MiRoCore(MiRo):
	def __init__(self, name=''):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self, name)

		# Topic subscriptions
		rospy.Subscriber(self.tr + 'core/animal/state', miro.msg.animal_state, self.callback_core_state)
		# rospy.Subscriber(topic_root + '/core/detect_objects_l', miro.msg.objects, self.callback_detect_objects_l)
		# rospy.Subscriber(topic_root + '/core/detect_objects_r', miro.msg.objects, self.callback_detect_objects_r)
		# rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
		# rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)
		# rospy.Subscriber(topic_root + '/core/detect_face_l', Float32MultiArray, self.callback_detect_face_l)
		# rospy.Subscriber(topic_root + '/core/detect_face_r', Float32MultiArray, self.callback_detect_face_r)
		rospy.Subscriber(self.tr + 'core/selection/priority', Float32MultiArray, self.callback_selection_priority)
		rospy.Subscriber(self.tr + 'core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)
		# NEW: Motivation
		rospy.Subscriber(self.tr + 'motivation', Float32MultiArray, self.callback_motivation)

		# Default data
		self.affect = None
		# self.core_detect_objects_l = None
		# self.core_detect_objects_r = None
		# self.core_detect_ball_l = None
		# self.core_detect_ball_r = None
		# self.core_detect_face_l = None
		# self.core_detect_face_r = None
		self.motivation = None
		self.selection_priority = None
		self.selection_inhibition = None
		self.time = None

		# Sleep for ROS initialisation
		rospy.sleep(self.sleep)

	def callback_core_state(self, data):
		# FIXME: Time of day seems to be integrated into state now
		self.affect = data

		# Convert 'time of day' value into a 12hr value with half-hour precision
		# TODO: There's surely a neater way of doing this
		self.time = round(0.5 * round((24 * data.time_of_day) / 0.5), 2)
		if self.time > 12.5:
			self.time = self.time - 12

	def callback_motivation(self, data):
		self.motivation = data

	def callback_selection_priority(self, data):
		self.selection_priority = data

	def callback_selection_inhibition(self, data):
		self.selection_inhibition = data


class MiRoPerception(MiRo):
	# Asynchronous sensors
	def __init__(self, name=''):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self, name)

		# TODO: Add test for physical or sim. robot
		self.opt = {'Uncompressed': False}

		# Topic subscriptions
		if self.opt['Uncompressed']:
			# TODO: Uncompressed callbacks not yet tested
			rospy.Subscriber(self.tr + 'sensors/caml', Image, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr', Image, self.callback_camr)
		else:
			rospy.Subscriber(self.tr + 'sensors/caml/compressed', CompressedImage, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr/compressed', CompressedImage, self.callback_camr)
		rospy.Subscriber(self.tr + 'sensors/mics', Int16MultiArray, self.callback_mics)
		# Saliency maps will only be available when MiRo is running in demo mode
		rospy.Subscriber(self.tr + 'core/pril', Image, self.callback_pril)
		rospy.Subscriber(self.tr + 'core/prir', Image, self.callback_prir)
		rospy.Subscriber(self.tr + 'core/priw', Image, self.callback_priw)

		# Default data
		self.caml = None
		self.camr = None
		self.mics = None
		self.pril = None
		self.prir = None
		self.priw = None

		# Sleep for ROS initialisation
		rospy.sleep(self.sleep)

	# TODO: Image stitching
	def callback_caml(self, frame):
		self.caml = self.process_frame(frame)

	def callback_camr(self, frame):
		self.camr = self.process_frame(frame)

	def callback_mics(self, msg):
		# TODO: Is this rescaling necessary? What does it achieve?
		# data = np.asarray(msg.data, 'float32') * (1.0 / 32768.0)
		data = np.asarray(msg.data, 'float32')
		data = data.reshape((con.MICS, con.BLOCK_SAMPLES))
		self.mics = {
			'left'  : data[0],
			'right' : data[1],
			'centre': data[2],
			'tail'  : data[3]
		}

	def callback_pril(self, frame):
		self.pril = self.process_pri(frame)

	def callback_prir(self, frame):
		self.prir = self.process_pri(frame)

	def callback_priw(self, frame):
		self.priw = self.process_priw(frame)

	@staticmethod
	def process_frame(frame):
		# Decode image from numpy string format
		frame_bgr = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)

		# Convert image to RGB order
		frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)

		# Convert to image format - default output is 640x360
		return Im.fromarray(frame_rgb)

		# TODO: Can we convert directly from source in a single step?
		# return Im.frombytes('RGB', (320, 176), np.fromstring(frame.data, np.uint8), 'raw')
		# return Im.frombytes('RGB', (182, 100), np.fromstring(frame.data, np.uint8), 'jpeg')

	@staticmethod
	def process_pri(frame):
		# TODO: Convert to image type with full alpha channel and make background transparent
		# TODO: Move these constants to constants.py
		# Get monochrome image
		pri = Im.frombytes('L', (178, 100), np.fromstring(frame.data, np.uint8), 'raw')

		# Invert image for overlaying
		return ImageOps.invert(pri)

	@staticmethod
	def process_priw(frame):
		# Get monochrome image
		return Im.frombytes('L', (256, 1), np.fromstring(frame.data, np.uint8), 'raw')


class MiRoSensors(MiRo):
	# Synchronous 50Hz sensors
	def __init__(self, name=''):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self, name)

		# Topic subscriptions
		rospy.Subscriber(self.tr + 'sensors/package', msg.sensors_package, self.callback_sensors)

		# Initialise data
		self.sensors = None
		self.kinematic_joints = None
		self.light = None
		self.cliff = None
		self.sonar = None

		# Sleep for ROS initialisation
		rospy.sleep(self.sleep)

	def callback_sensors(self, sensors):
		self.sensors = sensors

		# Internal degrees-of-freedom configuration (radians)
		# In the order [TILT, LIFT, YAW, PITCH]
		self.kinematic_joints = {
			'tilt' : sensors.kinematic_joints.position[0],
			'lift' : sensors.kinematic_joints.position[1],
			'yaw'  : sensors.kinematic_joints.position[2],
			'pitch': sensors.kinematic_joints.position[3]
		}

		# Normalised light level at each of the four light sensors
		# 0 = dark, 1 = lit
		# In the order [FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT]
		self.light = {
			'front_left' : sensors.light.data[0],
			'front_right': sensors.light.data[1],
			'rear_left'  : sensors.light.data[2],
			'rear_right' : sensors.light.data[3],
		}

		# Normalised reading from each of the two cliff sensors
		# 0 = no surface, 1 = surface
		# In the order [LEFT, RIGHT].
		self.cliff = {
			'left' : sensors.cliff.data[0],
			'right': sensors.cliff.data[1],
		}

		# Range to strongest sonar reflector (metres)
		# Normal values are in the interval [0.03, 1.0]
		self.sonar = sensors.sonar.range


class MiRoPublishers(MiRo):
	def __init__(self, name=''):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self, name)

		# Topics
		self.cmd_vel = rospy.Publisher(self.tr + 'control/cmd_vel', TwistStamped, queue_size=self.qs)
		self.kinematic_joints = rospy.Publisher(self.tr + 'control/kinematic_joints', JointState, queue_size=self.qs)
		self.cosmetic_joints = rospy.Publisher(self.tr + 'control/cosmetic_joints', Float32MultiArray, queue_size=self.qs)
		self.illum = rospy.Publisher(self.tr + 'control/illum', UInt32MultiArray, queue_size=self.qs)
		self.tone = rospy.Publisher(self.tr + 'control/tone', UInt16MultiArray, queue_size=self.qs)

		# Initialise messages
		self.cmd_vel_msg = TwistStamped()
		self.kinematic_joints_msg = JointState()
		self.cosmetic_joints_msg = Float32MultiArray()
		self.illum_msg = UInt32MultiArray()
		self.tone_msg = UInt16MultiArray()

		# Sleep for ROS initialisation
		rospy.sleep(self.sleep)

	# Publish wheel speeds (m/s)
	def pub_cmd_vel_ms(self, left=0, right=0):
		(dr, dtheta) = miro.utils.wheel_speed2cmd_vel([left, right])
		self.pub_cmd_vel_rad(dr, dtheta)

	# Publish wheel speeds (radians)
	def pub_cmd_vel_rad(self, dr, dtheta):
		self.cmd_vel_msg.twist.linear.x = dr
		self.cmd_vel_msg.twist.angular.z = dtheta
		self.cmd_vel.publish(self.cmd_vel_msg)

	# Publish cosmetic joints
	# TODO: Add 'all' kwarg to define all / multiple joints in single array
	def pub_cosmetic_joints(
			self,
			droop=0.5,
			wag=0.5,
			eye_left=0,
			eye_right=0,
			ear_left=0,
			ear_right=0
	):
		# Normalised configuration of cosmetic joints
		# Six joints are commanded in the order [DROOP, WAG, L EYE, R EYE, L EAR, R EAR]
		# Joint positions:
		# Droop: ?=up       ?=down
		# Wag:   0=left     1=right
		# Eyes:  0=open     1=closed
		# Ears:  0=inwards  1=outwards
		self.cosmetic_joints_msg.data = [droop, wag, eye_left, eye_right, ear_left, ear_right]
		self.cosmetic_joints.publish(self.cosmetic_joints_msg)

	# Publish kinematic joint positions
	# TODO: Convert these into more easily understandable values
	def pub_kinematic_joints(
			self,
			tilt=0,
			lift=0,
			yaw=0,
			pitch=0
	):
		# Internal DOF configuration (radians) in the order [TILT, LIFT, YAW, PITCH]
		self.kinematic_joints_msg.position = [tilt, lift, yaw, pitch]
		self.kinematic_joints.publish(self.kinematic_joints_msg)

	# Publish illumination
	def pub_illum(
			self,
			left_front=0x00000000,
			left_mid=0x00000000,
			left_rear=0x00000000,
			right_front=0x00000000,
			right_mid=0x00000000,
			right_rear=0x00000000,
			**kwargs
	):
		# Commanded pattern for the six LEDs in the order [L FRONT, L MIDDLE, L REAR, R FRONT, R MIDDLE, R REAR]
		# Each element is an ARGB word (0xAARRGGBB) where A is a brightness channel that scales the other three

		# Set basic named colours
		for key in kwargs:
			# TODO: Replace with switch/case when moving to Python 3
			if kwargs[key] == 'red':
				kwargs[key] = 0xFFFF0000
			elif kwargs[key] == 'green':
				kwargs[key] = 0xFF00FF00
			elif kwargs[key] == 'blue':
				kwargs[key] = 0xFF0000FF
			elif kwargs[key] == 'yellow':
				kwargs[key] = 0xFFFFFF00
			elif kwargs[key] == 'cyan':
				kwargs[key] = 0xFF00FFFF
			elif kwargs[key] == 'magenta':
				kwargs[key] = 0xFFFF00FF
			elif kwargs[key] == 'white':
				kwargs[key] = 0xFFFFFFFF

		# Set multiple lights based on keyword arguments
		if kwargs.get('front'):
			left_front = right_front = kwargs['front']

		if kwargs.get('mid'):
			left_mid = right_mid = kwargs['mid']

		if kwargs.get('rear'):
			left_rear = right_rear = kwargs['rear']

		if kwargs.get('left_all'):
			left_front = left_mid = left_rear = kwargs['left_all']

		if kwargs.get('right_all'):
			right_front = right_mid = right_rear = kwargs['right_all']

		if kwargs.get('all'):
			left_front = left_mid = left_rear = right_front = right_mid = right_rear = kwargs['all']

		self.illum_msg.data = [left_front, left_mid, left_rear, right_front, right_mid, right_rear]
		self.illum.publish(self.illum_msg)

	# Publish audio tone
	def pub_tone(
			self,
			frequency=0,
			volume=0,
			duration=0
	):
		# Frequency in Hz (values between 50 and 2000)
		frequency = np.clip(
			frequency,
			con.TONE_FREQUENCY['min'],
			con.TONE_FREQUENCY['max']
		)
		# Volume from 0 to 255
		volume = np.clip(
			volume,
			con.TONE_VOLUME['min'],
			con.TONE_VOLUME['max']
		)
		# Duration in platform ticks (20ms periods)
		duration = np.maximum(0, duration)

		self.tone_msg.data = [frequency, volume, duration]
		self.tone.publish(self.tone_msg)
