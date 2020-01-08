# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Image, Imu, Range, CompressedImage

# Image handling
import cv2
from PIL import Image as Im
from PIL import ImageOps

# Other imports
import os
import numpy as np
import miro2 as miro
import rospy

# Set publisher queue size
QUEUE_SIZE = 10


class MiroClient:
    def __init__(self):

        # TODO: Add test for physical or sim. robot
        self.opt = {'Uncompressed': False}

        # Set topic root
        topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Initialise ROS node
        rospy.init_node("MiRo_ROS_python", anonymous=True)

        # SUBSCRIBERS
        # TODO: Detect face
        # TODO: Detect ball
        # Core: Cognitive state (Demo mode only)
        rospy.Subscriber(topic_root + '/core/animal/state', miro.msg.animal_state, self.callback_core_state)
        rospy.Subscriber(topic_root + '/core/pril', Image, self.callback_pril)
        rospy.Subscriber(topic_root + '/core/prir', Image, self.callback_prir)
        rospy.Subscriber(topic_root + '/core/priw', Image, self.callback_priw)
        # rospy.Subscriber(topic_root + '/core/detect_objects_l', miro.msg.objects, self.callback_detect_objects_l)
        # rospy.Subscriber(topic_root + '/core/detect_objects_r', miro.msg.objects, self.callback_detect_objects_r)
        # rospy.Subscriber(topic_root + '/core/detect_ball_l', UInt16MultiArray, self.callback_detect_ball_l)
        # rospy.Subscriber(topic_root + '/core/detect_ball_r', UInt16MultiArray, self.callback_detect_ball_r)
        # rospy.Subscriber(topic_root + '/core/detect_face_l', Float32MultiArray, self.callback_detect_face_l)
        # rospy.Subscriber(topic_root + '/core/detect_face_r', Float32MultiArray, self.callback_detect_face_r)
        rospy.Subscriber(topic_root + '/core/selection/priority', Float32MultiArray, self.callback_selection_priority)
        rospy.Subscriber(topic_root + '/core/selection/inhibition', Float32MultiArray, self.callback_selection_inhibition)

        # Sensors
        if self.opt['Uncompressed']:
            # TODO: Uncompressed callbacks not yet tested
            rospy.Subscriber(topic_root + '/sensors/caml', Image, self.callback_caml)
            rospy.Subscriber(topic_root + '/sensors/camr', Image, self.callback_camr)
        else:
            rospy.Subscriber(topic_root + '/sensors/caml/compressed', CompressedImage, self.callback_caml)
            rospy.Subscriber(topic_root + '/sensors/camr/compressed', CompressedImage, self.callback_camr)
        rospy.Subscriber(topic_root + '/sensors/kinematic_joints', JointState, self.callback_kinematic_joints)

        # Default data
        self.core_affect = None
        self.core_pril = None
        self.core_prir = None
        self.core_priw = None
        # self.core_detect_objects_l = None
        # self.core_detect_objects_r = None
        # self.core_detect_ball_l = None
        # self.core_detect_ball_r = None
        # self.core_detect_face_l = None
        # self.core_detect_face_r = None
        self.core_time = None
        self.selection_priority = None
        self.selection_inhibition = None
        self.sensors_caml = None
        self.sensors_camr = None
        self.sensors_kinematic_joints = {}

        # PUBLISHERS
        self.cmd_vel = rospy.Publisher(topic_root + '/control/cmd_vel', TwistStamped, queue_size=QUEUE_SIZE)
        self.cosmetic_joints = rospy.Publisher(topic_root + '/control/cosmetic_joints', Float32MultiArray, queue_size=QUEUE_SIZE)
        self.kinematic_joints = rospy.Publisher(topic_root + '/control/kinematic_joints', JointState, queue_size=QUEUE_SIZE)
        self.illum = rospy.Publisher(topic_root + '/control/illum', UInt32MultiArray, queue_size=QUEUE_SIZE)

        # Initialise messages
        self.illum_msg = UInt32MultiArray()
        self.kinematic_joints_msg = JointState()
        self.kinematic_joints_msg.name = ['tilt', 'lift', 'yaw', 'pitch']
        self.velocity_msg = TwistStamped()

    # SUBSCRIBER callbacks
    # Core
    def callback_core_state(self, data):
        # FIXME: Time of day seems to be integrated into state now
        self.core_affect = data

        # Convert 'time of day' value into a 12hr value with half-hour precision
        # TODO: There's surely a neater way of doing this
        self.core_time = round(0.5 * round((24 * data.time_of_day) / 0.5), 2)
        if self.core_time > 12.5:
            self.core_time = self.core_time - 12

    # def callback_detect_objects_l(self, data):
    # 	self.core_detect_objects_l = data
    #
    # def callback_detect_objects_r(self, data):
    # 	self.core_detect_objects_r = data

    # def callback_detect_ball_l(self, data):
    # 	self.core_detect_ball_l = data
    #
    # def callback_detect_ball_r(self, data):
    # 	self.core_detect_ball_r = data
    #
    # def callback_detect_face_l(self, data):
    # 	self.core_detect_face_l = data
    #
    # def callback_detect_face_r(self, data):
    # 	self.core_detect_face_r = data

    def callback_selection_priority(self, data):
        self.selection_priority = data

    def callback_selection_inhibition(self, data):
        self.selection_inhibition = data

    # def callback_core_time(self, data):
    # 	self.core_time = data

    def callback_pril(self, frame):
        self.core_pril = self.process_pri(frame)

    def callback_prir(self, frame):
        self.core_prir = self.process_pri(frame)

    def callback_priw(self, frame):
        self.core_priw = self.process_priw(frame)

    # Sensors
    # TODO: Image stitching before passing images back to dashboard
    def callback_caml(self, frame):
        self.sensors_caml = self.process_frame(frame)

    def callback_camr(self, frame):
        self.sensors_camr = self.process_frame(frame)

    def callback_kinematic_joints(self, data):
        for n in range(len(data.name)):
            self.sensors_kinematic_joints[data.name[n]] = data.position[n]

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

    # PUBLISHER functions
    # # Publish eyelid positions (1 = closed, 0 = open)
    # def pub_cosmetic_eyes(self, eye_l, eye_r):
    #     # Initialise cosmetic joints
    #     cos_joints = Float32MultiArray()
    #     # FIXME: Get current value for other cosmetic joints
    #     # droop, wag, eyel, eyer, earl, earr
    #     cos_joints.data = [0, 0.5, eye_l, eye_r, 0.3, 0.3]
    #
    #     self.pub_cos.publish(cos_joints)

<<<<<<< Updated upstream
    # Publish kinematic joint positions
    def pub_kinematic(self, tilt, lift, yaw, pitch):
        # Construct ROS message
        kinematic_joints = JointState()
        kinematic_joints.name = ['tilt', 'lift', 'yaw', 'pitch']
        kinematic_joints.position = [tilt, lift, yaw, pitch]

        # Publish
        self.pub_kin.publish(kinematic_joints)
=======
    # Publish illumination
    # TODO: May be more convenient to pass this as a single list?
    def pub_illum(self, left_front, left_mid, left_rear, right_front, right_mid, right_rear):
        # Each element is an ARGB word where the alpha channel scales the other three
        self.illum_msg.data = [left_front, left_mid, left_rear, right_front, right_mid, right_rear]
        self.illum.publish(self.illum_msg)

    # Publish kinematic joint positions
    def pub_kinematic_joints(self, tilt, lift, yaw, pitch):
        # Internal DOF configuration (Rad) in the order [TILT, LIFT, YAW, PITCH]
        self.kinematic_joints_msg.position = [tilt, lift, yaw, pitch]
        self.kinematic_joints.publish(self.kinematic_joints_msg)
>>>>>>> Stashed changes

    # Publish wheel speeds (m/s)
    def pub_cmd_vel(self, whl_l, whl_r):
        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = miro.utils.wheel_speed2cmd_vel([whl_l, whl_r])

<<<<<<< Updated upstream
        # Construct ROS message
        velocity = TwistStamped()
        velocity.twist.linear.x = dr
        velocity.twist.angular.z = dtheta

        # Publish
        self.pub_cmd_vel.publish(velocity)
=======
        # Construct and publish ROS message
        self.velocity_msg.twist.linear.x = dr
        self.velocity_msg.twist.angular.z = dtheta
        self.cmd_vel.publish(self.velocity_msg)
>>>>>>> Stashed changes
