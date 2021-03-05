# MiRo-E ROS interfaces
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from miro2_msg import msg

# MiRo-E modules and parameters
import constants as con
import miro2 as miro

# Image handling
import cv2
from PIL import Image as Im
from PIL import ImageOps

# Other imports
import os
import math
import numpy as np
import rosnode
import rospy




class MiRo:
	def __init__(self):
		name = 'MiRo_ROS_interface'
		if name not in rosnode.get_node_names():
			# Initialise ROS node ('disable_rostime=True' needed to work in PyCharm)
			rospy.init_node(name, anonymous=True, disable_rostime="PYCHARM_HOSTED" in os.environ)

		self.tr = '/' + os.getenv('MIRO_ROBOT_NAME') + '/'  # ROS topic root
		self.qs = 2                                         # Publisher queue size
		self.sleep = 0.5                                    # Initialisation sleep duration


class MiRoCore(MiRo):
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

		# Topic subscriptions
		# State
		rospy.Subscriber(self.tr + 'core/animal/state', miro.msg.animal_state, self.callback_core_state)
		# rospy.Subscriber(topic_root + '/core/detect_objects_l', miro.msg.objects, self.callback_detect_objects_l)
		# rospy.Subscriber(topic_root + '/core/detect_objects_r', miro.msg.objects, self.callback_detect_objects_r)
		# rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
		# rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)
		# rospy.Subscriber(topic_root + '/core/detect_face_l', Float32MultiArray, self.callback_detect_face_l)
		# rospy.Subscriber(topic_root + '/core/detect_face_r', Float32MultiArray, self.callback_detect_face_r)
		# Salience maps
		rospy.Subscriber(self.tr + 'core/pril', Image, self.callback_pril)
		rospy.Subscriber(self.tr + 'core/prir', Image, self.callback_prir)
		rospy.Subscriber(self.tr + 'core/priw', Image, self.callback_priw)
		# Selection
		rospy.Subscriber(self.tr + 'core/selection/priority', Float32MultiArray, self.callback_selection_priority)
		rospy.Subscriber(self.tr + 'core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)
		# Motivation
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
		self.pril = None
		self.prir = None
		self.priw = None
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

	def callback_pril(self, frame):
		self.pril = self.process_pri(frame)

	def callback_prir(self, frame):
		self.prir = self.process_pri(frame)

	def callback_priw(self, frame):
		self.priw = self.process_priw(frame)

	def callback_selection_priority(self, data):
		self.selection_priority = data

	def callback_selection_inhibition(self, data):
		self.selection_inhibition = data

	@staticmethod
	def process_pri(frame):
		# TODO: Convert to image type with full alpha channel and make background transparent
		# Get monochrome image
		pri = Im.frombytes('L', (con.PRI['width'], con.PRI['height']), np.fromstring(frame.data, np.uint8), 'raw')

		# Invert image for overlaying
		return ImageOps.invert(pri)

	@staticmethod
	def process_priw(frame):
		# Get monochrome image
		return Im.frombytes('L', (con.PRIW['width'], con.PRIW['height']), np.fromstring(frame.data, np.uint8), 'raw')


class MiRoPerception(MiRo):
	# Asynchronous sensors
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

		# TODO: Add test for physical or simulated robot to switch this flag
		self.opt = {'Uncompressed': False}

		# Topic subscriptions
		if self.opt['Uncompressed']:
			# TODO: Uncompressed callbacks are untested
			rospy.Subscriber(self.tr + 'sensors/caml', Image, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr', Image, self.callback_camr)
		else:
			rospy.Subscriber(self.tr + 'sensors/caml/compressed', CompressedImage, self.callback_caml)
			rospy.Subscriber(self.tr + 'sensors/camr/compressed', CompressedImage, self.callback_camr)
		rospy.Subscriber(self.tr + 'sensors/mics', Int16MultiArray, self.callback_mics)

		# Default data
		self.caml = None
		self.caml_undistorted = None
		self.camr = None
		self.camr_undistorted = None
		self.mics = None

		# Sleep for ROS initialisation
		rospy.sleep(self.sleep)

	# TODO: Image stitching
	def callback_caml(self, frame):
		# self.caml = self.process_frame(frame)
		image = self.process_frame(frame)
		self.caml = image['normal']
		self.caml_undistorted = image['undistorted']

	def callback_camr(self, frame):
		# self.camr = self.process_frame(frame)
		image = self.process_frame(frame)
		self.camr = image['normal']
		self.camr_undistorted = image['undistorted']

	def callback_mics(self, msg):
		# Rescale data to be between -1 and +1
		data = np.asarray(msg.data, 'float32') * (1.0 / 32768.0)

		# Separate out samples from each microphone
		# Samples in the order [LEFT, RIGHT, CENTRE, TAIL]
		data = data.reshape((con.MICS, con.BLOCK_SAMPLES))
		self.mics = {
			'left'  : data[0],
			'right' : data[1],
			'centre': data[2],
			'tail'  : data[3]
		}

	@staticmethod
	def process_frame(frame):
		# Decode image from numpy string format
		frame_out = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)

		# Convert image to RGB order
		# frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)

		# TODO: Can we convert directly from source in a single step?
		# return Im.frombytes('RGB', (320, 176), np.fromstring(frame.data, np.uint8), 'raw')
		# return Im.frombytes('RGB', (182, 100), np.fromstring(frame.data, np.uint8), 'jpeg')

		# Undistort image
		frame_undistorted = cv2.undistort(frame_out, con.MTX, con.DIST, None)

		# Convert to image format - default output for unmodified frame is 640x360
		# return Im.fromarray(frame_rgb)
		return {'normal': frame_out, 'undistorted': frame_undistorted}


class MiRoSensors(MiRo):
	# Synchronous 50Hz sensors
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

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
	def __init__(self):
		# TODO: Use super() when moving to Python 3
		MiRo.__init__(self)

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

	# Publish cosmetic joint positions
	# TODO: Add 'all' kwarg to define all / multiple joints in single array
	def pub_cosmetic_joints(
			self,
			droop=con.DROOP,
			wag=con.WAG,
			eye_left=con.EYE,
			eye_right=con.EYE,
			ear_left=con.EAR,
			ear_right=con.EAR,
			**kwargs
	):
		# Normalised configuration of cosmetic joints
		# Six joints are commanded in the order [DROOP, WAG, L EYE, R EYE, L EAR, R EAR]
		# Joint positions:
		# Droop: 0=up       1=down
		# Wag:   0=left     1=right
		# Eyes:  0=open     1=closed
		# Ears:  0=inwards  1=outwards

		# Set multiple joints at once
		if kwargs.get('eyes'):
			eye_left = eye_right = kwargs['eyes']
		if kwargs.get('ears'):
			ear_left = ear_right = kwargs['ears']

		self.cosmetic_joints_msg.data = [droop, wag, eye_left, eye_right, ear_left, ear_right]
		self.cosmetic_joints.publish(self.cosmetic_joints_msg)

	# Publish kinematic joint positions
	# TODO: Convert these into more easily understandable values
	def pub_kinematic_joints(
			self,
			lift=con.LIFT['calib'],
			yaw=con.YAW['calib'],
			pitch=con.PITCH['calib']
	):
		# Internal DOF configuration (radians) in the order [TILT, LIFT, YAW, PITCH]
		self.kinematic_joints_msg.position = [
			math.radians(con.TILT),
			math.radians(
				np.clip(
					lift,
					con.LIFT['min'],
					con.LIFT['max']
				)
			),
			math.radians(
				np.clip(
					yaw,
					con.YAW['min'],
					con.YAW['max']
				)
			),
			math.radians(
				np.clip(
					pitch,
					con.PITCH['min'],
					con.PITCH['max']
				)
			),
		]

		self.kinematic_joints.publish(self.kinematic_joints_msg)

	# Publish illumination
	def pub_illum(
			self,
			left_front=con.ARGB_WORD['off'],
			left_mid=con.ARGB_WORD['off'],
			left_rear=con.ARGB_WORD['off'],
			right_front=con.ARGB_WORD['off'],
			right_mid=con.ARGB_WORD['off'],
			right_rear=con.ARGB_WORD['off'],
			**kwargs
	):
		# Commanded pattern for the six LEDs in the order [L FRONT, L MIDDLE, L REAR, R FRONT, R MIDDLE, R REAR]
		# Each element is an ARGB word (0xAARRGGBB) where A is a brightness channel that scales the other three
		for key in kwargs:
			try:
				# If a named colour is specified try to retrieve it from the colour list
				kwargs[key] = con.ARGB_WORD[kwargs[key]]
			except KeyError:
				if isinstance(kwargs[key], str):
					print('Unknown colour name provided')
				else:
					# Assume we received an ARGB word
					# TODO: Add better error checking here (eg. if user tries to pass an RGB tuple)
					pass

		# Set multiple lights at once
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
			frequency=None,
			volume=None,
			duration=None,
			note=None
	):
		# Convert a string note (eg. "A4") to a frequency (eg. 440)
		# From https://gist.github.com/CGrassin/26a1fdf4fc5de788da9b376ff717516e - MIT license
		def get_frequency(note, A4=440):
			notes = ['A', 'A#', 'B', 'C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#']

			octave = int(note[2]) if len(note) == 3 else int(note[1])

			key_number = notes.index(note[0:-1])

			if key_number < 3:
				key_number = key_number + 12 + ((octave - 1) * 12) + 1
			else:
				key_number = key_number + ((octave - 1) * 12) + 1

			return A4 * 2 ** ((key_number - 49) / 12)

		# A note value overrides the specified frequency
		if note is not None:
			try:
				frequency = get_frequency(note)
			except IndexError:
				print('Please specify both note and octave value, eg. C4')
			except ValueError:
				print('{} is not a recognised note'.format(note))

		# Frequency in Hz (values between 50 and 2000)
		frequency = int(np.clip(
			frequency,
			con.TONE_FREQUENCY['min'],
			con.TONE_FREQUENCY['max']
		))

		# Volume from 0 to 255
		volume = np.clip(
			volume,
			con.TONE_VOLUME['min'],
			con.TONE_VOLUME['max']
		)

		# Duration in seconds (20ms platform ticks * 50)
		duration = np.maximum(0, duration * 50)

		self.tone_msg.data = [frequency, volume, duration]
		self.tone.publish(self.tone_msg)
