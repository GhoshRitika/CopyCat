# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge, CvBridgeError
# from enum import Enum, auto
# import pyrealsense2 as rs
# import cv2
# import mediapipe as mp
# import numpy as np
# import math

# JOINT_LIMIT_THUMB_HIGH = 0.0
# JOINT_LIMIT_THUMB_LOW = 0.0
# JOINT_LIMIT_INDEX_HIGH = 0.0
# JOINT_LIMIT_INDEX_LOW = 0.0
# JOINT_LIMIT_MIDDLE_HIGH = 0.0
# JOINT_LIMIT_MIDDLE_LOW = 0.0
# JOINT_LIMIT_RING_HIGH = 0.0
# JOINT_LIMIT_RING_LOW =  0.0


# class FingerTracking(Node):
#     def __init__(self):
#         super().__init__('finger_tracking')
#         self.mp_drawing = mp.solutions.drawing_utils
#         self.mp_drawing_styles = mp.solutions.drawing_styles
#         self.mp_hands = mp.solutions.hands
#         # self.cap = cv2.VideoCapture(0)
#         # self.hands = self.mp_hands.Hands(
#         #     model_complexity=0,
#         #     min_detection_confidence=0.5,
#         #     min_tracking_confidence=0.5)
#         self.angles = Float64MultiArray()
#         # self.data = []
#         self.bridge = CvBridge()
#         self.intrinsics = None
#         self.color_frame = None
#         self.depth_frame = None

#         self.color_sub = self.create_subscription(Image, '/camera/color/image_raw',
#                                                     self.color_callback, 10)
#         self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
#                                                     self.depth_callback, 10)
#         self.info_sub = self.create_subscription(CameraInfo,
#                                                  "/camera/aligned_depth_to_color/camera_info",
#                                                  self.info_callback, 10)
#         self.ang_pub = self.create_publisher(Float64MultiArray, 'Joint_angles', 10)
#         self.timer = self.create_timer(0.01, self.timer_callback)

#     def info_callback(self, cameraInfo):
#         """Store the intrinsics of the camera."""
#         try:
#             if self.intrinsics:
#                 return
#             self.intrinsics = rs.intrinsics()
#             self.intrinsics.width = cameraInfo.width
#             self.intrinsics.height = cameraInfo.height
#             # self.intrinsics.ppx = cameraInfo.k[2]
#             # self.intrinsics.ppy = cameraInfo.k[5]
#             # self.intrinsics.fx = cameraInfo.k[0]
#             # self.intrinsics.fy = cameraInfo.k[4]
#         except CvBridgeError:
#             self.get_logger().info("Getting intrinsics failed?")
#             return

#     def color_callback(self, data):
#         self.get_logger().info("Getting frames")
#         self.color_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
#     def depth_callback(self, data):
#         self.depth_frame=self.bridge.imgmsg_to_cv2(data)
#         #put a mask on it
#         depth_copy = np.asanyarray(self.depth_frame)
#         color_copy = np.asanyarray(self.color_frame)
#         # depth_sensor = profile.get_device().first_depth_sensor()
#         # depth_scale = depth_sensor.get_depth_scale()
#         # print("Depth Scale is: ", depth_scale)

#         #  (THIS IS REDUNDANT, I am going to delete it)
#         # clipping_distance_in_meters = 2.5  # 1 meter
#         # clipping_distance = clipping_distance_in_meters / depth_scale
#         # depth_mask = cv2.inRange(np.array(depth_copy), band_start,
#         #                          band_start+band_width)
#         grey = 153
#         clip = 5.0 * 1000
#         depth_frame_3d = np.dstack((depth_copy, depth_copy, depth_copy))
#         self.bg_removed = np.where((depth_frame_3d < clip) |
#                                 (depth_frame_3d <= 0), grey, color_copy)
#         # color_mask = cv2.inRange(np.array(color_copy), 1, 225)
#         # This operation helps to remove "dots" on the depth image.
#         # Kernel higher dimensional = smoother. It's also less important if camera is farther away.
#         # kernel = np.ones((25, 25), np.uint8)
#         # self.depth_mask = cv2.morphologyEx(bg_removed, cv2.MORPH_CLOSE, kernel)

#         self.hands = self.mp_hands.Hands(
#             model_complexity=0,
#             min_detection_confidence=0.5,
#             min_tracking_confidence=0.5)
    
#     def get_indices(self, handLandmarks):
#         self.wrist = [handLandmarks.landmark[0].x, handLandmarks.landmark[0].y]

#         self.thumb_cmc = [handLandmarks.landmark[1].x, handLandmarks.landmark[1].y]
#         self.thumb_mcp = [handLandmarks.landmark[2].x, handLandmarks.landmark[2].y]
#         self.thumb_ip = [handLandmarks.landmark[3].x, handLandmarks.landmark[3].y]
#         self.thumb_tip = [handLandmarks.landmark[4].x, handLandmarks.landmark[4].y]

#         self.index_mcp = [handLandmarks.landmark[5].x, handLandmarks.landmark[5].y]
#         self.index_pip = [handLandmarks.landmark[6].x, handLandmarks.landmark[6].y]
#         self.index_dip = [handLandmarks.landmark[7].x, handLandmarks.landmark[7].y]
#         self.index_tip = [handLandmarks.landmark[8].x, handLandmarks.landmark[8].y]

#         self.middle_mcp = [handLandmarks.landmark[9].x, handLandmarks.landmark[9].y]
#         self.middle_pip = [handLandmarks.landmark[10].x, handLandmarks.landmark[10].y]
#         self.middle_dip = [handLandmarks.landmark[11].x, handLandmarks.landmark[11].y]
#         self.middle_tip = [handLandmarks.landmark[12].x, handLandmarks.landmark[12].y]

#         self.ring_mcp = [handLandmarks.landmark[13].x, handLandmarks.landmark[13].y]
#         self.ring_pip = [handLandmarks.landmark[14].x, handLandmarks.landmark[14].y]
#         self.ring_dip = [handLandmarks.landmark[15].x, handLandmarks.landmark[15].y]
#         self.ring_tip = [handLandmarks.landmark[16].x, handLandmarks.landmark[16].y]


#     def calc_angle(self):
#         m0 =  math.atan2(self.wrist[1] - self.thumb_cmc[1], (self.wrist[0] - self.thumb_cmc[0]))
#         m1 =  math.atan2(self.thumb_mcp[1] - self.thumb_cmc[1], (self.thumb_mcp[0] - self.thumb_cmc[0]))
#         m2 =  math.atan2(self.thumb_ip[1] - self.thumb_mcp[1], (self.thumb_ip[0] - self.thumb_mcp[0]))
#         m3 =  math.atan2(self.thumb_tip[1] - self.thumb_ip[1], (self.thumb_tip[0] - self.thumb_ip[0]))

#         m4 =  math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[0]))
#         m5 =  math.atan2(self.index_pip[1] - self.index_mcp[1], (self.index_pip[0] - self.index_mcp[0]))
#         m6 =  math.atan2(self.index_dip[1] - self.index_pip[1], (self.index_dip[0] - self.index_pip[0]))
#         m7 =  math.atan2(self.index_tip[1] - self.index_dip[1], (self.index_tip[0] - self.index_dip[0]))

#         m8 =  math.atan2(self.wrist[1] - self.middle_mcp[1], (self.wrist[0] - self.middle_mcp[0]))
#         m9 =  math.atan2(self.middle_pip[1] - self.middle_mcp[1], (self.middle_pip[0] - self.middle_mcp[0]))
#         m10 =  math.atan2(self.middle_dip[1] - self.middle_pip[1], (self.middle_dip[0] - self.middle_pip[0]))
#         m11 =  math.atan2(self.middle_tip[1] - self.middle_dip[1], (self.middle_tip[0] - self.middle_dip[0]))

#         m12 =  math.atan2(self.wrist[1] - self.ring_mcp[1], (self.wrist[0] - self.ring_mcp[0]))
#         m13 =  math.atan2(self.ring_pip[1] - self.ring_mcp[1], (self.ring_pip[0] - self.ring_mcp[0]))
#         m14 =  math.atan2(self.ring_dip[1] - self.ring_pip[1], (self.ring_dip[0] - self.ring_pip[0]))
#         m15 =  math.atan2(self.ring_tip[1] - self.ring_dip[1], (self.ring_tip[0] - self.ring_dip[0]))

#         m16 = math.atan2(self.wrist[1] - self.thumb_cmc[1], (self.wrist[0] - self.thumb_cmc[1]))
#         m17 = math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[1]))


#         self.thumb_wrist = math.atan(abs((m0 - m1) / (1 + m0 * m1))) - 0.8
#         self.thumb_index = math.atan(abs((m16 - m17) / (1 + m16 * m17)))
#         self.thumb_base = math.atan(abs((m1 - m2) / (1 + m1 * m2))) 
#         self.thumb_knuckle = math.atan(abs((m2 - m3) / (1 + m2 * m3)))

#         self.index_base = math.atan(abs((m4 - m5) / (1 + m4 * m5))) - 0.8
#         self.index_knuckle = math.atan(abs((m5 - m6) / (1 + m5 * m6)))
#         self.index_tip = math.atan(abs((m6 - m7) / (1 + m6 * m7))) 

#         self.middle_base =  math.atan(abs((m8 - m9) / (1 + m8 * m9))) - 0.8
#         self.middle_knuckle = math.atan(abs((m9 - m10) / (1 + m9 * m10)))
#         self.middle_tip = math.atan(abs((m10 - m11) / (1 + m10 * m11))) 

#         self.ring_base = math.atan(abs((m12 - m13) / (1 + m12 * m13))) - 0.8
#         self.ring_knuckle = math.atan(abs((m13 - m14) / (1 + m13 * m14)))
#         self.ring_tip = math.atan(abs((m14 - m15) / (1 + m14 * m15))) 


#     def timer_callback(self):
#         # self.cap = cv2.VideoCapture(0)
#         # with self.mp_hands.Hands(
#         #     model_complexity=0,
#         #     min_detection_confidence=0.5,
#         #     min_tracking_confidence=0.5) as hands:
#             # if self.cap.isOpened():
#             #     success, image = self.cap.read()
#             #     if not success:
#             #         print("Ignoring empty camera frame.")
#             #         # If loading a video, use 'break' instead of 'continue'.
#                 if self.depth_frame is not None:
#                     image = self.bg_removed
#                     # To improve performance, optionally mark the image as not writeable to
#                     # pass by reference.
#                     image.flags.writeable = False
#                     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#                     results = self.hands.process(image)

#                     # Draw the hand annotations on the image.
#                     image.flags.writeable = True
#                     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#                     if results.multi_hand_landmarks:
#                         # since we will be dealing with only one hand
#                         for hand_landmarks in results.multi_hand_landmarks:
#                             self.get_indices(hand_landmarks)
#                             self.calc_angle()
#                             # self.angles.data = [0.0,self.index_base, self.index_knuckle, self.index_tip,
#                             # 0.0, self.middle_knuckle, self.middle_tip, self.middle_base,
#                             # 0.0,  self.ring_knuckle,self.ring_tip,self.ring_base, 
#                             # self.thumb_wrist, 0.0, self.thumb_base, self.thumb_knuckle,]
#                             self.angles.data = [0.0, self.index_base, self.index_knuckle, self.index_tip,
#                             0.0, self.middle_base, self.middle_knuckle, self.middle_tip,
#                         0.0, self.ring_base, self.ring_knuckle, self.ring_tip, 
#                             self.thumb_wrist, self.thumb_index, self.thumb_base, self.thumb_knuckle]
#                             self.ang_pub.publish(self.angles)
#                             self.mp_drawing.draw_landmarks(
#                                 image,
#                                 hand_landmarks,
#                                 self.mp_hands.HAND_CONNECTIONS,
#                                 self.mp_drawing_styles.get_default_hand_landmarks_style(),
#                                 self.mp_drawing_styles.get_default_hand_connections_style())
#                     # Flip the image horizontally for a selfie-view display.
#                     cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
#                     if cv2.waitKey(5) & 0xFF == 27:
#                         cv2.destroyAllWindows()
#                     #throw execption
#                     # self.cap.release()
#         # self.cap.release()

# def main(args=None):
#     rclpy.init(args=args)
#     node = FingerTracking()
#     rclpy.spin(node)
#     node.cap.release()
#     rclpy.shutdown()
#     # cv2.destroyAllWindows()
#     # self.cap.release()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
import math
import numpy as np
# import tensorflow as tf
# import * as handPoseDetection from '@tensorflow-models/hand-pose-detection' 

JOINT_LIMIT_THUMB_HIGH = 0.0
JOINT_LIMIT_THUMB_LOW = 0.0
JOINT_LIMIT_INDEX_HIGH = 0.0
JOINT_LIMIT_INDEX_LOW = 0.0
JOINT_LIMIT_MIDDLE_HIGH = 0.0
JOINT_LIMIT_MIDDLE_LOW = 0.0
JOINT_LIMIT_RING_HIGH = 0.0
JOINT_LIMIT_RING_LOW =  0.0


class FingerTracking(Node):
    def __init__(self):
        super().__init__('finger_tracking')
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.cap = cv2.VideoCapture(0)
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        # self.model = handPoseDetection.SupportedModels.MediaPipeHands
        # self.detectorConfig = {
        #     runtime: 'mediapipe',
        #     modelType: 'full'
        # }
        # self.detector = await handPoseDetection.createDetector(model, detectorConfig)
        self.angles = Float64MultiArray()
        self.mid_pose = PoseStamped()
        # self.data = []
        self.ang_pub = self.create_publisher(Float64MultiArray, 'Joint_angles', 10)
        self.middle_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)


    def get_indices(self, handLandmarks):
        self.wrist = [handLandmarks.landmark[0].x, handLandmarks.landmark[0].y, handLandmarks.landmark[0].z]

        self.thumb_cmc = [handLandmarks.landmark[1].x, handLandmarks.landmark[1].y, handLandmarks.landmark[1].z]
        self.thumb_mcp = [handLandmarks.landmark[2].x, handLandmarks.landmark[2].y, handLandmarks.landmark[2].z]
        self.thumb_ip = [handLandmarks.landmark[3].x, handLandmarks.landmark[3].y, handLandmarks.landmark[3].z]
        self.thumb_tip = [handLandmarks.landmark[4].x, handLandmarks.landmark[4].y, handLandmarks.landmark[4].z]

        self.index_mcp = [handLandmarks.landmark[5].x, handLandmarks.landmark[5].y, handLandmarks.landmark[5].z]
        self.index_pip = [handLandmarks.landmark[6].x, handLandmarks.landmark[6].y, handLandmarks.landmark[6].z]
        self.index_dip = [handLandmarks.landmark[7].x, handLandmarks.landmark[7].y, handLandmarks.landmark[7].z]
        self.index_tip = [handLandmarks.landmark[8].x, handLandmarks.landmark[8].y, handLandmarks.landmark[8].z]

        self.middle_mcp = [handLandmarks.landmark[9].x, handLandmarks.landmark[9].y, handLandmarks.landmark[9].z]
        self.middle_pip = [handLandmarks.landmark[10].x, handLandmarks.landmark[10].y, handLandmarks.landmark[10].z]
        self.middle_dip = [handLandmarks.landmark[11].x, handLandmarks.landmark[11].y, handLandmarks.landmark[11].z]
        self.middle_tip = [handLandmarks.landmark[12].x, handLandmarks.landmark[12].y, handLandmarks.landmark[12].z]

        self.ring_mcp = [handLandmarks.landmark[13].x, handLandmarks.landmark[13].y, handLandmarks.landmark[13].z]
        self.ring_pip = [handLandmarks.landmark[14].x, handLandmarks.landmark[14].y, handLandmarks.landmark[14].z]
        self.ring_dip = [handLandmarks.landmark[15].x, handLandmarks.landmark[15].y, handLandmarks.landmark[15].z]
        self.ring_tip = [handLandmarks.landmark[16].x, handLandmarks.landmark[16].y, handLandmarks.landmark[16].z]

        self.thumb_tip[0] = self.thumb_tip[0] - self.middle_mcp[0]
        self.thumb_tip[1] = self.thumb_tip[1] - self.middle_mcp[1]
        self.thumb_tip[2] = self.thumb_tip[2] - self.middle_mcp[2]
        

        self.index_tip[0] = self.index_tip[0] - self.middle_mcp[0]
        self.index_tip[1] = self.index_tip[1] - self.middle_mcp[1]
        self.index_tip[2] = self.index_tip[2] - self.middle_mcp[2]

        self.middle_tip[0] = self.middle_tip[0] - self.middle_mcp[0]
        self.middle_tip[1] = self.middle_tip[1] - self.middle_mcp[1]
        self.middle_tip[2] = self.middle_tip[2] - self.middle_mcp[2]
        print("relative middle", self.middle_tip)

        self.ring_tip[0] = self.ring_tip[0] - self.middle_mcp[0]
        self.ring_tip[1] = self.ring_tip[1] - self.middle_mcp[1]
        self.ring_tip[2] = self.ring_tip[2] - self.middle_mcp[2]


    def calc_angle(self):
        # m0 =  math.atan2(self.wrist[1] - self.thumb_cmc[1], (self.wrist[0] - self.thumb_cmc[0]))
        # m1 =  math.atan2(self.thumb_mcp[1] - self.thumb_cmc[1], (self.thumb_mcp[0] - self.thumb_cmc[0]))
        # m2 =  math.atan2(self.thumb_ip[1] - self.thumb_mcp[1], (self.thumb_ip[0] - self.thumb_mcp[0]))
        # m3 =  math.atan2(self.thumb_tip[1] - self.thumb_ip[1], (self.thumb_tip[0] - self.thumb_ip[0]))

        # m4 =  math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[0]))
        # m5 =  math.atan2(self.index_pip[1] - self.index_mcp[1], (self.index_pip[0] - self.index_mcp[0]))
        # m6 =  math.atan2(self.index_dip[1] - self.index_pip[1], (self.index_dip[0] - self.index_pip[0]))
        # m7 =  math.atan2(self.index_tip[1] - self.index_dip[1], (self.index_tip[0] - self.index_dip[0]))

        # m8 =  math.atan2(self.wrist[1] - self.middle_mcp[1], (self.wrist[0] - self.middle_mcp[0]))
        # m9 =  math.atan2(self.middle_pip[1] - self.middle_mcp[1], (self.middle_pip[0] - self.middle_mcp[0]))
        # m10 =  math.atan2(self.middle_dip[1] - self.middle_pip[1], (self.middle_dip[0] - self.middle_pip[0]))
        # m11 =  math.atan2(self.middle_tip[1] - self.middle_dip[1], (self.middle_tip[0] - self.middle_dip[0]))

        # m12 =  math.atan2(self.wrist[1] - self.ring_mcp[1], (self.wrist[0] - self.ring_mcp[0]))
        # m13 =  math.atan2(self.ring_pip[1] - self.ring_mcp[1], (self.ring_pip[0] - self.ring_mcp[0]))
        # m14 =  math.atan2(self.ring_dip[1] - self.ring_pip[1], (self.ring_dip[0] - self.ring_pip[0]))
        # m15 =  math.atan2(self.ring_tip[1] - self.ring_dip[1], (self.ring_tip[0] - self.ring_dip[0]))

        m0 =  [(self.wrist[0] - self.thumb_cmc[0]), (self.wrist[1] - self.thumb_cmc[1]),(self.wrist[2] - self.thumb_cmc[2])]
        m1 =  [(self.thumb_mcp[0] - self.thumb_cmc[0]), (self.thumb_mcp[1] - self.thumb_cmc[1]), (self.thumb_mcp[2] - self.thumb_cmc[2])]
        m2 =  [(self.thumb_ip[0] - self.thumb_mcp[0]), (self.thumb_ip[1] - self.thumb_mcp[1]), (self.thumb_ip[2] - self.thumb_mcp[2])]
        m3 =  [(self.thumb_tip[0] - self.thumb_ip[0]), (self.thumb_tip[1] - self.thumb_ip[1]), (self.thumb_tip[2] - self.thumb_ip[2])]

        m4 =  [(self.wrist[0] - self.index_mcp[0]), (self.wrist[1] - self.index_mcp[1]), (self.wrist[2] - self.index_mcp[2])]
        m5 =  [(self.index_pip[0] - self.index_mcp[0]), (self.index_pip[1] - self.index_mcp[1]), (self.index_pip[2] - self.index_mcp[2])]
        m6 =  [(self.index_dip[0] - self.index_pip[0]), (self.index_dip[1] - self.index_pip[1]), (self.index_dip[2] - self.index_pip[2])]
        m7 =  [(self.index_tip[0] - self.index_dip[0]), (self.index_tip[1] - self.index_dip[1]), (self.index_tip[2] - self.index_dip[2])]

        m8 =  [(self.wrist[0] - self.middle_mcp[0]), (self.wrist[1] - self.middle_mcp[1]), (self.wrist[2] - self.middle_mcp[2])]
        m9 =  [(self.middle_pip[0] - self.middle_mcp[0]), (self.middle_pip[1] - self.middle_mcp[1]), (self.middle_pip[2] - self.middle_mcp[2])]
        m10 = [(self.middle_dip[0] - self.middle_pip[0]), (self.middle_dip[1] - self.middle_pip[1]), (self.middle_dip[2] - self.middle_pip[2])]
        m11 =  [(self.middle_tip[0] - self.middle_dip[0]), (self.middle_tip[1] - self.middle_dip[1]), (self.middle_tip[2] - self.middle_dip[2])]

        m12 =  [(self.wrist[0] - self.ring_mcp[0]), (self.wrist[1] - self.ring_mcp[1]), (self.wrist[2] - self.ring_mcp[2])]
        m13 =  [(self.ring_pip[0] - self.ring_mcp[0]), (self.ring_pip[1] - self.ring_mcp[1]), (self.ring_pip[2] - self.ring_mcp[2])]
        m14 =  [(self.ring_dip[0] - self.ring_pip[0]), (self.ring_dip[1] - self.ring_pip[1]), (self.ring_dip[2] - self.ring_pip[2])]
        m15 =  [(self.ring_tip[0] - self.ring_dip[0]), (self.ring_tip[1] - self.ring_dip[1]), (self.ring_tip[2] - self.ring_dip[2])]

        # m16 = math.atan2(self.self.thumb_mcp[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[1]))


        self.thumb_wrist = 1.5708 - np.arccos((m0[0]*m1[0] + m0[1]*m1[1] + m0[0]*m1[2]) / ((m0[0]**2 + m0[1]**2 + m0[2]**2)*(m1[0]**2 + m1[1]**2 + m1[2]**2))**0.5)
        self.thumb_base = np.arccos((m2[0]*m1[0] + m2[1]*m1[1] + m2[0]*m1[2]) / ((m1[0]**2 + m1[1]**2 + m1[2]**2)*(m2[0]**2 + m2[1]**2 + m2[2]**2))**0.5)
        self.thumb_knuckle = np.arccos((m2[0]*m3[0] + m2[1]*m3[1] + m2[0]*m3[2]) / ((m3[0]**2 + m3[1]**2 + m3[2]**2)*(m2[0]**2 + m2[1]**2 + m2[2]**2))**0.5)

        self.index_base = 1.5708 - np.arccos((m4[0]*m5[0] + m4[1]*m5[1] + m4[0]*m5[2]) / ((m4[0]**2 + m4[1]**2 + m4[2]**2)*(m5[0]**2 + m5[1]**2 + m5[2]**2))**0.5)
        self.index_knuckle = np.arccos((m6[0]*m5[0] + m6[1]*m5[1] + m6[0]*m5[2]) / ((m6[0]**2 + m6[1]**2 + m6[2]**2)*(m5[0]**2 + m5[1]**2 + m5[2]**2))**0.5)
        self.index_tip = np.arccos((m6[0]*m7[0] + m6[1]*m7[1] + m6[0]*m7[2]) / ((m6[0]**2 + m6[1]**2 + m6[2]**2)*(m7[0]**2 + m7[1]**2 + m7[2]**2))**0.5)

        self.middle_base = 1.5708 - np.arccos((m8[0]*m9[0] + m8[1]*m9[1] + m8[0]*m9[2]) / ((m8[0]**2 + m8[1]**2 + m8[2]**2)*(m9[0]**2 + m9[1]**2 + m9[2]**2))**0.5)
        self.middle_knuckle = np.arccos((m10[0]*m9[0] + m10[1]*m9[1] + m10[0]*m9[2]) / ((m10[0]**2 + m10[1]**2 + m10[2]**2)*(m9[0]**2 + m9[1]**2 + m9[2]**2))**0.5)
        self.middle_tip = np.arccos((m10[0]*m11[0] + m10[1]*m11[1] + m10[0]*m11[2]) / ((m10[0]**2 + m10[1]**2 + m10[2]**2)*(m11[0]**2 + m11[1]**2 + m11[2]**2))**0.5)

        self.ring_base = 1.5708 - np.arccos((m12[0]*m13[0] + m12[1]*m13[1] + m12[0]*m13[2]) / ((m12[0]**2 + m12[1]**2 + m12[2]**2)*(m13[0]**2 + m13[1]**2 + m13[2]**2))**0.5)
        self.ring_knuckle =  np.arccos((m14[0]*m13[0] + m14[1]*m13[1] + m14[0]*m13[2]) / ((m14[0]**2 + m14[1]**2 + m14[2]**2)*(m13[0]**2 + m13[1]**2 + m13[2]**2))**0.5)
        self.ring_tip = np.arccos((m14[0]*m15[0] + m14[1]*m15[1] + m14[0]*m15[2]) / ((m14[0]**2 + m14[1]**2 + m14[2]**2)*(m15[0]**2 + m15[1]**2 + m15[2]**2))**0.5)


    def timer_callback(self):
        # self.cap = cv2.VideoCapture(0)
        # with self.mp_hands.Hands(
        #     model_complexity=0,
        #     min_detection_confidence=0.5,
        #     min_tracking_confidence=0.5) as hands:
            if self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # if results.multi_hand_world_landmarks:
                if results.multi_hand_landmarks:
                    # since we will be dealing with only one hand
                    # 
                    for hand_landmarks in results.multi_hand_landmarks:
                        
                        self.get_indices(hand_landmarks)
                        # self.calc_angle()
                        # self.angles.data = [0.0, self.index_base, self.index_knuckle, self.index_tip,
                        # 0.0, self.middle_base, self.middle_knuckle, self.middle_tip,
                        # 0.0, self.ring_base, self.ring_knuckle, self.ring_tip,
                        # self.thumb_wrist, 0.0, self.thumb_base, self.thumb_knuckle,]
                        # self.ang_pub.publish(self.angles)
                        # self.angles.data = [self.index_tip, self.thumb_tip, self.middle_tip, self.ring_tip]
                        self.mid_pose.pose.position.x = self.middle_tip[2]
                        self.mid_pose.pose.position.y = -self.middle_tip[0]
                        self.mid_pose.pose.position.z = (-self.middle_tip[1]/0.25)*0.1095
                        # print(self.mid_pose)
                        # self.ang_pub.publish(self.angles)
                        self.middle_pub.publish(self.mid_pose)
                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())
                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                    cv2.destroyAllWindows()
                    #throw execption
                    # self.cap.release()
        # self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = FingerTracking()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()
    # cv2.destroyAllWindows()
    # self.cap.release()