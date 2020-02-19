# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage
from miro2_msg import msg

# Image handling
import cv2
from PIL import Image as Im
from PIL import ImageOps

# Other imports
import os
import rospy
import base64
import numpy as np
import miro2 as miro
from io import BytesIO


class MiRo:
	def __init__(self):
		# Initialise ROS node
		# 'disable_rostime=True' needed to work in Pycharm
		rospy.init_node("MiRo_ROS_interface", anonymous=True, disable_rostime=True)

		# Set topic root
		self.tr = '/' + os.getenv('MIRO_ROBOT_NAME') + '/'

		# Set publisher queue size
		self.qs = 10


class MiRoCore(MiRo):
	def __init__(self):
		# TODO: Use super()
		MiRo.__init__(self)

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
	def __init__(self):
		# TODO: Use super()
		MiRo.__init__(self)

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
		rospy.Subscriber(self.tr + 'core/pril', Image, self.callback_pril)
		rospy.Subscriber(self.tr + 'core/prir', Image, self.callback_prir)
		rospy.Subscriber(self.tr + 'core/priw', Image, self.callback_priw)

		# Default data
		self.caml = None
		self.camr = None
		self.pril = None
		self.prir = None
		self.priw = None

		self.caml_b64 = None
		self.camr_b64 = None

		self.time = None

	# TODO: Image stitching before passing images back to dashboard
	def callback_caml(self, frame):
		self.caml = self.process_frame(frame)
		# self.caml_b64 = self.frame_b64(self.caml)

	def callback_camr(self, frame):
		self.camr = self.process_frame(frame)
		# self.camr_b64 = self.frame_b64(self.camr)

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

		# Convert to image format
		return Im.fromarray(frame_rgb)

		# TODO: Can we convert directly from source in one go?
		# return Im.frombytes('RGB', (320, 176), np.fromstring(frame.data, np.uint8), 'raw')
		# return Im.frombytes('RGB', (182, 100), np.fromstring(frame.data, np.uint8), 'jpeg')

	# # TODO: Decide whether to keep this function here or move it to Dashboard app
	# @staticmethod
	# def frame_b64(frame):
	# 	# Create base64 URI from image: https://stackoverflow.com/questions/16065694/is-it-possible-to-create-encoded-base64-url-from-image-object
	# 	frame_buffer = BytesIO()
	# 	frame_sml = frame.resize((320, 180))
	# 	frame_sml.save(frame_buffer, format='PNG')
	# 	frame_b64 = base64.b64encode(frame_buffer.getvalue())
	#
	# 	return 'data:image/png;base64,{}'.format(frame_b64)

	@staticmethod
	def process_pri(frame):
		# Get monochrome image
		pri = Im.frombytes('L', (178, 100), np.fromstring(frame.data, np.uint8), 'raw')

		# TODO: Convert to image type with full alpha channel and make background transparent
		# Invert image for overlaying
		return ImageOps.invert(pri)

	@staticmethod
	def process_priw(frame):
		# Get monochrome image
		return Im.frombytes('L', (1, 256), np.fromstring(frame.data, np.uint8), 'raw')


class MiRoSensors(MiRo):
	def __init__(self):
		# TODO: Use super()
		MiRo.__init__(self)

		# Topic subscriptions
		rospy.Subscriber(self.tr + 'sensors', msg.sensors_package, self.callback_sensors)

		# Default data
		# TODO: Split here into separate sensors
		self.sensors = None

	def callback_sensors(self, data):
		self.sensors = data


# class MiRoPublishers(MiRo):
# 	def __init__(self):
# 		# TODO: Use super()
# 		MiRo.__init__(self)
#
# 		# Set publisher queue size
# 		self.qs = 10
#
# 		# Topics
# 		self.cmd_vel = rospy.Publisher(self.tr + 'control/cmd_vel', TwistStamped, queue_size=self.qs)
#
# 	# Publish wheel speeds (m/s)
# 	def pub_cmd_vel_ms(self, whl_l, whl_r):
# 		# Convert wheel speed to command velocity and publish
# 		(dr, dtheta) = miro.utils.wheel_speed2cmd_vel([whl_l, whl_r])
# 		self.pub_cmd_vel_rad(dr, dtheta)
#
# 	# Publish wheel speeds (radians)
# 	def pub_cmd_vel_rad(self, dr, dtheta):
# 		self.cmd_vel_msg.twist.linear.x = dr
# 		self.cmd_vel_msg.twist.angular.z = dtheta
# 		self.cmd_vel.publish(self.cmd_vel_msg)


# # TODO: Split into clients for each data type (e.g. video, sensors)
# class MiroClient:
#     def __init__(self):
#
#         # TODO: Add test for physical or sim. robot
#         self.opt = {'Uncompressed': False}
#
#         # Set topic root
#         topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
#
#         # Initialise ROS node
#         # 'disable_rostime' must be True to work in Pycharm
#         # rospy.init_node("MiRo_ROS_interface", anonymous=True, disable_rostime=True)
#
#         # SUBSCRIBERS
#         # TODO: Detect face
#         # TODO: Detect ball
#         # Core: Cognitive state (Demo mode only)
#         rospy.Subscriber(topic_root + '/core/animal/state', miro.msg.animal_state, self.callback_core_state)
#         rospy.Subscriber(topic_root + '/core/pril', Image, self.callback_pril)
#         rospy.Subscriber(topic_root + '/core/prir', Image, self.callback_prir)
#         rospy.Subscriber(topic_root + '/core/priw', Image, self.callback_priw)
#         # rospy.Subscriber(topic_root + '/core/detect_objects_l', miro.msg.objects, self.callback_detect_objects_l)
#         # rospy.Subscriber(topic_root + '/core/detect_objects_r', miro.msg.objects, self.callback_detect_objects_r)
#         # rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
#         # rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)
#         # rospy.Subscriber(topic_root + '/core/detect_face_l', Float32MultiArray, self.callback_detect_face_l)
#         # rospy.Subscriber(topic_root + '/core/detect_face_r', Float32MultiArray, self.callback_detect_face_r)
#         rospy.Subscriber(topic_root + '/core/selection/priority', Float32MultiArray, self.callback_selection_priority)
#         rospy.Subscriber(topic_root + '/core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)
#         # NEW: Motivation
#         rospy.Subscriber(topic_root + '/motivation', Float32MultiArray, self.callback_motivation)
#
#         # Sensors
#         if self.opt['Uncompressed']:
#             # TODO: Uncompressed callbacks not yet tested
#             rospy.Subscriber(topic_root + '/sensors/caml', Image, self.callback_caml)
#             rospy.Subscriber(topic_root + '/sensors/camr', Image, self.callback_camr)
#         else:
#             rospy.Subscriber(topic_root + '/sensors/caml/compressed', CompressedImage, self.callback_caml)
#             rospy.Subscriber(topic_root + '/sensors/camr/compressed', CompressedImage, self.callback_camr)
#         rospy.Subscriber(topic_root + '/sensors/kinematic_joints', JointState, self.callback_kinematic_joints)
#
#         # Default data
#         self.core_affect = None
#         self.core_pril = None
#         self.core_prir = None
#         self.core_priw = None
#         # self.core_detect_objects_l = None
#         # self.core_detect_objects_r = None
#         # self.core_detect_ball_l = None
#         # self.core_detect_ball_r = None
#         # self.core_detect_face_l = None
#         # self.core_detect_face_r = None
#         self.core_time = None
#         self.core_motivation = None
#         self.selection_priority = None
#         self.selection_inhibition = None
#         self.sensors_caml = None
#         self.sensors_camr = None
#         self.sensors_kinematic_joints = {}
#
#         # # PUBLISHERS
#         # self.cmd_vel = rospy.Publisher(topic_root + '/control/cmd_vel', TwistStamped, queue_size=QUEUE_SIZE)
#         # self.cosmetic_joints = rospy.Publisher(topic_root + '/control/cosmetic_joints', Float32MultiArray, queue_size=QUEUE_SIZE)
#         # self.kinematic_joints = rospy.Publisher(topic_root + '/control/kinematic_joints', JointState, queue_size=QUEUE_SIZE)
#         # self.illum = rospy.Publisher(topic_root + '/control/illum', UInt32MultiArray, queue_size=QUEUE_SIZE)
# 		#
#         # # Initialise messages
#         # self.cmd_vel_msg = TwistStamped()
#         # self.kinematic_joints_msg = JointState()
#         # self.kinematic_joints_msg.name = ['tilt', 'lift', 'yaw', 'pitch']
#         # self.illum_msg = UInt32MultiArray()
#
#     # SUBSCRIBER callbacks
#     # Core
#     def callback_core_state(self, data):
#         # FIXME: Time of day seems to be integrated into state now
#         self.core_affect = data
#
#         # Convert 'time of day' value into a 12hr value with half-hour precision
#         # TODO: There's surely a neater way of doing this
#         self.core_time = round(0.5 * round((24 * data.time_of_day) / 0.5), 2)
#         if self.core_time > 12.5:
#             self.core_time = self.core_time - 12
#
#     # def callback_detect_objects_l(self, data):
#     # 	self.core_detect_objects_l = data
#     #
#     # def callback_detect_objects_r(self, data):
#     # 	self.core_detect_objects_r = data
#
#     # def callback_detect_ball_l(self, data):
#     # 	self.core_detect_ball_l = data
#     #
#     # def callback_detect_ball_r(self, data):
#     # 	self.core_detect_ball_r = data
#     #
#     # def callback_detect_face_l(self, data):
#     # 	self.core_detect_face_l = data
#     #
#     # def callback_detect_face_r(self, data):
#     # 	self.core_detect_face_r = data
#
#     def callback_selection_priority(self, data):
#         self.selection_priority = data
#
#     def callback_selection_inhibition(self, data):
#         self.selection_inhibition = data
#
#     # def callback_core_time(self, data):
#     # 	self.core_time = data
#
#     def callback_pril(self, frame):
#         self.core_pril = self.process_pri(frame)
#
#     def callback_prir(self, frame):
#         self.core_prir = self.process_pri(frame)
#
#     def callback_priw(self, frame):
#         self.core_priw = self.process_priw(frame)
#
#     # Sensors
#     # TODO: Image stitching before passing images back to dashboard
#     def callback_caml(self, frame):
#         self.sensors_caml = self.process_frame(frame)
#
#     def callback_camr(self, frame):
#         self.sensors_camr = self.process_frame(frame)
#
#     def callback_kinematic_joints(self, data):
#         for n in range(len(data.name)):
#             self.sensors_kinematic_joints[data.name[n]] = data.position[n]
#
#     def callback_motivation(self, data):
#         self.core_motivation = data
#
#     @staticmethod
#     def process_frame(frame):
#         # Decode image from numpy string format
#         frame_bgr = cv2.imdecode(np.fromstring(frame.data, np.uint8), cv2.IMREAD_COLOR)
#
#         # Convert image to RGB order
#         frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)
#
#         # Convert to image format
#         return Im.fromarray(frame_rgb)
#
#     # TODO: Can we convert directly from source in one go?
#     # return Im.frombytes('RGB', (320, 176), np.fromstring(frame.data, np.uint8), 'raw')
#     # return Im.frombytes('RGB', (182, 100), np.fromstring(frame.data, np.uint8), 'jpeg')
#
#     @staticmethod
#     def process_pri(frame):
#         # Get monochrome image
#         pri = Im.frombytes('L', (178, 100), np.fromstring(frame.data, np.uint8), 'raw')
#
#         # TODO: Convert to image type with full alpha channel and make background transparent
#         # Invert image for overlaying
#         return ImageOps.invert(pri)
#
#     @staticmethod
#     def process_priw(frame):
#         # Get monochrome image
#         return Im.frombytes('L', (1, 256), np.fromstring(frame.data, np.uint8), 'raw')
#
#     # PUBLISHER functions
#     # # Publish eyelid positions (1 = closed, 0 = open)
#     # def pub_cosmetic_eyes(self, eye_l, eye_r):
#     #     # Initialise cosmetic joints
#     #     cos_joints = Float32MultiArray()
#     #     # FIXME: Get current value for other cosmetic joints
#     #     # droop, wag, eyel, eyer, earl, earr
#     #     cos_joints.data = [0, 0.5, eye_l, eye_r, 0.3, 0.3]
#     #
#     #     self.pub_cos.publish(cos_joints)
#
#     # Publish illumination
#     # TODO: May be more convenient to pass this as a single list?
#     def pub_illum(self, left_front, left_mid, left_rear, right_front, right_mid, right_rear):
#         # Each element is an ARGB word where the alpha channel scales the other three
#         self.illum_msg.data = [left_front, left_mid, left_rear, right_front, right_mid, right_rear]
#         self.illum.publish(self.illum_msg)
#
#     # Publish kinematic joint positions
#     def pub_kinematic_joints(self, tilt, lift, yaw, pitch):
#         # Internal DOF configuration (Rad) in the order [TILT, LIFT, YAW, PITCH]
#         self.kinematic_joints_msg.position = [tilt, lift, yaw, pitch]
#         self.kinematic_joints.publish(self.kinematic_joints_msg)
#
#     # Publish wheel speeds (m/s)
#     def pub_cmd_vel_ms(self, whl_l, whl_r):
#         # Convert wheel speed to command velocity and publish
#         (dr, dtheta) = miro.utils.wheel_speed2cmd_vel([whl_l, whl_r])
#         self.pub_cmd_vel_rad(dr, dtheta)
#
#     # Publish wheel speeds (radians)
#     def pub_cmd_vel_rad(self, dr, dtheta):
#         self.cmd_vel_msg.twist.linear.x = dr
#         self.cmd_vel_msg.twist.angular.z = dtheta
#         self.cmd_vel.publish(self.cmd_vel_msg)
