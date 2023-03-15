import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import cv2
import mediapipe as mp
import math
import numpy as np


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
        self.angles = Float64MultiArray()
        self.mid_pose = PoseStamped()
        self.ang_pub = self.create_publisher(Float64MultiArray, 'Joint_angles', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.timer2 = self.create_timer(2.5, self.timer_callback2)


    def get_indices(self, handLandmarks):
        self.wrist = [0.0, 0.0, 0.0]

        self.thumb_cmc = [handLandmarks.landmark[1].x - handLandmarks.landmark[0].x, handLandmarks.landmark[1].y - handLandmarks.landmark[0].y, handLandmarks.landmark[1].z - handLandmarks.landmark[0].z]
        self.thumb_mcp = [handLandmarks.landmark[2].x - handLandmarks.landmark[0].x, handLandmarks.landmark[2].y - handLandmarks.landmark[0].y, handLandmarks.landmark[2].z - handLandmarks.landmark[0].z]
        self.thumb_ip = [handLandmarks.landmark[3].x - handLandmarks.landmark[0].x, handLandmarks.landmark[3].y - handLandmarks.landmark[0].y, handLandmarks.landmark[3].z - handLandmarks.landmark[0].z]
        self.thumb_tip = [handLandmarks.landmark[4].x - handLandmarks.landmark[0].x, handLandmarks.landmark[4].y - handLandmarks.landmark[0].y, handLandmarks.landmark[4].z - handLandmarks.landmark[0].z]

        self.index_mcp = [handLandmarks.landmark[5].x - handLandmarks.landmark[0].x, handLandmarks.landmark[5].y - handLandmarks.landmark[0].y, handLandmarks.landmark[5].z - handLandmarks.landmark[0].z]
        self.index_pip = [handLandmarks.landmark[6].x - handLandmarks.landmark[0].x, handLandmarks.landmark[6].y - handLandmarks.landmark[0].y, handLandmarks.landmark[6].z - handLandmarks.landmark[0].z]
        self.index_dip = [handLandmarks.landmark[7].x - handLandmarks.landmark[0].x, handLandmarks.landmark[7].y - handLandmarks.landmark[0].y, handLandmarks.landmark[7].z - handLandmarks.landmark[0].z]
        self.index_tip = [handLandmarks.landmark[8].x - handLandmarks.landmark[0].x, handLandmarks.landmark[8].y - handLandmarks.landmark[0].y, handLandmarks.landmark[8].z - handLandmarks.landmark[0].z]

        self.middle_mcp = [handLandmarks.landmark[9].x - handLandmarks.landmark[0].x, handLandmarks.landmark[9].y - handLandmarks.landmark[0].y, handLandmarks.landmark[9].z - handLandmarks.landmark[0].z]
        self.middle_pip = [handLandmarks.landmark[10].x - handLandmarks.landmark[0].x, handLandmarks.landmark[10].y - handLandmarks.landmark[0].y, handLandmarks.landmark[10].z - handLandmarks.landmark[0].z]
        self.middle_dip = [handLandmarks.landmark[11].x - handLandmarks.landmark[0].x, handLandmarks.landmark[11].y - handLandmarks.landmark[0].y, handLandmarks.landmark[11].z - handLandmarks.landmark[0].z]
        self.middle_tip = [handLandmarks.landmark[12].x - handLandmarks.landmark[0].x, handLandmarks.landmark[12].y - handLandmarks.landmark[0].y, handLandmarks.landmark[12].z - handLandmarks.landmark[0].z]

        self.ring_mcp = [handLandmarks.landmark[13].x - handLandmarks.landmark[0].x, handLandmarks.landmark[13].y - handLandmarks.landmark[0].y, handLandmarks.landmark[13].z - handLandmarks.landmark[0].z]
        self.ring_pip = [handLandmarks.landmark[14].x - handLandmarks.landmark[0].x, handLandmarks.landmark[14].y - handLandmarks.landmark[0].y, handLandmarks.landmark[14].z - handLandmarks.landmark[0].z]
        self.ring_dip = [handLandmarks.landmark[15].x - handLandmarks.landmark[0].x, handLandmarks.landmark[15].y - handLandmarks.landmark[0].y, handLandmarks.landmark[15].z - handLandmarks.landmark[0].z]
        self.ring_tip = [handLandmarks.landmark[16].x - handLandmarks.landmark[0].x, handLandmarks.landmark[16].y - handLandmarks.landmark[0].y, handLandmarks.landmark[16].z - handLandmarks.landmark[0].z]

    def calc_angle(self):
        self.index_knuckle = self.GetAngleABC(self.index_dip, self.index_pip, self.index_mcp)
        if(self.index_knuckle > 120):
            self.index_knuckle = 120
        elif (self.index_knuckle<5):
            self.index_knuckle=5
        self.index_knuckle = (self.index_knuckle-5) * (0-95)/(5-120)

        self.index_base = self.GetAngleABC(self.index_pip, self.index_mcp, self.wrist) 
        if(self.index_base > 35):
            self.index_base = 35
        elif (self.index_base<5):
            self.index_base=5
        self.index_base = (self.index_base-5) * (0-90)/(5-35)

        self.index_top = self.GetAngleABC(self.index_tip, self.index_dip, self.index_pip)
        if(self.index_top > 35):
            self.index_top = 35
        elif (self.index_top<0):
            self.index_top=0
        self.index_top = (self.index_top) * (0-90)/(0-35)

        m4 =  math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[0]))
        m5 =  math.atan2(self.index_pip[1] - self.index_mcp[1], (self.index_pip[0] - self.index_mcp[0]))
        self.index_twist = math.atan(abs((m4 - m5) / (1 + m4 * m5))) *180.0/ 3.141592653589793
        if(self.index_twist > 70):
            self.index_twist = 70
        elif (self.index_twist<60):
            self.index_twist=60
        self.index_twist = (self.index_twist-65) * (-27-27)/(60-70)

        self.thumb_wrist = self.GetAngleABC(self.index_mcp, self.wrist, self.thumb_ip)
        if(self.thumb_wrist> 165):
            self.thumb_wrist= 165
        elif(self.thumb_wrist< 145):
            self.thumb_wrist=145
        self.thumb_wrist = 15 + (self.thumb_wrist - 145)* (15-80)/(145-165)

        m16 = math.atan2(self.wrist[1] - self.thumb_ip[1], (self.wrist[0] - self.thumb_ip[0]))
        m17 = math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[0]))
        self.thumb_index = 15 - math.atan(abs((m16 - m17) / (1 + m16 * m17))) *180.0/ 3.141592653589793
        if(self.thumb_index> 15):
            self.thumb_index= 15
        elif(self.thumb_index< 5):
            self.thumb_index=5
        self.thumb_index = -10 + (self.thumb_index - 5)* (-10-95)/(5-15)

        self.thumb_twist = self.GetAngleABC(self.thumb_ip, self.thumb_mcp, self.thumb_cmc)
        if(self.thumb_twist> 30):
            self.thumb_twist= 30
        elif(self.thumb_twist< 5):
            self.thumb_twist=5
        self.thumb_twist = -6 + (self.thumb_twist - 5)* (-6-67)/(5-30)

        self.thumb_knuckle = self.GetAngleABC(self.thumb_tip, self.thumb_ip, self.thumb_mcp)
        if(self.thumb_knuckle > 25):
            self.thumb_knuckle = 25
        elif(self.thumb_knuckle < 2):
            self.thumb_knuckle = 2
        self.thumb_knuckle = (self.thumb_knuckle-2)* (0-95)/(2-25)

        self.middle_base = self.GetAngleABC(self.middle_pip, self.middle_mcp, self.wrist) 
        if(self.middle_base > 60):
            self.middle_base = 60
        elif (self.middle_base<0):
            self.middle_base=0
        self.middle_base = (self.middle_base) * (0-90)/(0-60)
        
        m10 =  math.atan2(self.wrist[1] - self.middle_mcp[1], (self.wrist[0] - self.middle_mcp[0]))
        m11 =  math.atan2(self.middle_pip[1] - self.middle_mcp[1], (self.middle_pip[0] - self.middle_mcp[0]))
        self.middle_twist = math.atan(abs((m10 - m11) / (1 + m10 * m11))) *180.0/ 3.141592653589793
        if(self.middle_twist > 70):
            self.middle_twist = 70
        elif (self.middle_twist<60):
            self.middle_twist=60
        self.middle_twist = (self.middle_twist-65) * (-27-27)/(60-70)

        self.middle_knuckle = self.GetAngleABC(self.middle_dip, self.middle_pip, self.middle_mcp)
        if(self.middle_knuckle > 140):
            self.middle_knuckle = 140
        elif (self.middle_knuckle<5):
            self.middle_knuckle=5
        self.middle_knuckle = (self.middle_knuckle-5) * (0-95)/(5-140)

        self.middle_top = self.GetAngleABC(self.middle_tip, self.middle_dip, self.middle_pip)
        if(self.middle_top > 15):
            self.middle_top = 15
        elif (self.middle_top<0):
            self.middle_top=0
        self.middle_top = (self.middle_top) * (0-90)/(0-15)

        self.ring_base = self.GetAngleABC(self.ring_pip, self.ring_mcp, self.wrist) 
        if(self.ring_base > 60):
            self.ring_base = 60
        elif (self.ring_base<0):
            self.ring_base=0
        self.ring_base = (self.ring_base) * (0-90)/(0-60)

        m1 =  math.atan2(self.wrist[1] - self.ring_mcp[1], (self.wrist[0] - self.ring_mcp[0]))
        m2 =  math.atan2(self.ring_pip[1] - self.ring_mcp[1], (self.ring_pip[0] - self.ring_mcp[0]))
        self.ring_twist = math.atan(abs((m1 - m2) / (1 + m1 * m2))) *180.0/ 3.141592653589793
        if(self.ring_twist > 70):
            self.ring_twist = 70
        elif (self.ring_twist<60):
            self.ring_twist=60
        self.ring_twist = (self.ring_twist-65) * (-27-27)/(60-70)

        self.ring_knuckle = self.GetAngleABC(self.ring_dip, self.ring_pip, self.ring_mcp)
        if(self.ring_knuckle > 140):
            self.ring_knuckle = 140
        elif (self.ring_knuckle<5):
            self.ring_knuckle=5
        self.ring_knuckle = (self.ring_knuckle-5) * (0-95)/(5-140)

        self.ring_top = self.GetAngleABC(self.ring_tip, self.ring_dip, self.ring_pip)
        if(self.ring_top > 15):
            self.ring_top = 15
        elif (self.ring_top<0):
            self.ring_top=0
        self.ring_top = (self.ring_top) * (0-90)/(0-15)

        self.angles.data=[self.index_twist*0.0174533, self.index_base*0.0174533, self.index_knuckle*0.0174533, self.index_top*0.0174533,
                     self.thumb_wrist*0.0174533, self.thumb_twist*0.0174533, self.thumb_index*0.0174533, self.thumb_knuckle*0.0174533,
                     self.middle_twist*0.0174533, self.middle_base*0.0174533, self.middle_knuckle*0.0174533, self.middle_top*0.0174533,
                     self.ring_twist*0.0174533, self.ring_base*0.0174533, self.ring_knuckle*0.0174533, self.ring_top*0.0174533]


    def GetAngleABC(self, a, b, c):
        ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
        bc = [c[0] - b[0], c[1] - b[1], c[2] - b[2]]

        abVec = (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2])**0.5
        bcVec = (bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2])**0.5
        try:
            abNorm = [ab[0] / abVec, ab[1] / abVec, ab[2] / abVec]
            bcNorm = [bc[0] / bcVec, bc[1] / bcVec, bc[2] / bcVec]
        except ZeroDivisionError:
            abNorm = [ab[0] / (abVec+0.001), ab[1] / (abVec+0.001), ab[2] / (abVec+0.001)]
            bcNorm = [bc[0] / (bcVec+0.001), bc[1] / (bcVec+0.001), bc[2] / (bcVec+0.001)]

        res = abNorm[0] * bcNorm[0] + abNorm[1] * bcNorm[1] + abNorm[2] * bcNorm[2]
        
        return np.arccos(res)*180.0/ 3.141592653589793

    def timer_callback(self):
            if self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                # if results.multi_hand_world_landmarks:
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        
                        self.get_indices(hand_landmarks)
                        self.calc_angle()
                        # print(self.ring_top)
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

    def timer_callback2(self):
        print("PUBLISHING")
        self.ang_pub.publish(self.angles)

def main(args=None):
    rclpy.init(args=args)
    node = FingerTracking()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()
    # cv2.destroyAllWindows()
    # self.cap.release()