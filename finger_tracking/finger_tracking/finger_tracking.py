import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import math

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
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        self.angles = Float64MultiArray()
        # self.data = []
        self.ang_pub = self.create_publisher(Float64MultiArray, 'Joint_angles', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)


    def get_indices(self, handLandmarks):
        self.wrist = [handLandmarks.landmark[0].x, handLandmarks.landmark[0].y]

        self.thumb_cmc = [handLandmarks.landmark[1].x, handLandmarks.landmark[1].y]
        self.thumb_mcp = [handLandmarks.landmark[2].x, handLandmarks.landmark[2].y]
        self.thumb_ip = [handLandmarks.landmark[3].x, handLandmarks.landmark[3].y]
        self.thumb_tip = [handLandmarks.landmark[4].x, handLandmarks.landmark[4].y]

        self.index_mcp = [handLandmarks.landmark[5].x, handLandmarks.landmark[5].y]
        self.index_pip = [handLandmarks.landmark[6].x, handLandmarks.landmark[6].y]
        self.index_dip = [handLandmarks.landmark[7].x, handLandmarks.landmark[7].y]
        self.index_tip = [handLandmarks.landmark[8].x, handLandmarks.landmark[8].y]

        self.middle_mcp = [handLandmarks.landmark[9].x, handLandmarks.landmark[9].y]
        self.middle_pip = [handLandmarks.landmark[10].x, handLandmarks.landmark[10].y]
        self.middle_dip = [handLandmarks.landmark[11].x, handLandmarks.landmark[11].y]
        self.middle_tip = [handLandmarks.landmark[12].x, handLandmarks.landmark[12].y]

        self.ring_mcp = [handLandmarks.landmark[13].x, handLandmarks.landmark[13].y]
        self.ring_pip = [handLandmarks.landmark[14].x, handLandmarks.landmark[14].y]
        self.ring_dip = [handLandmarks.landmark[15].x, handLandmarks.landmark[15].y]
        self.ring_tip = [handLandmarks.landmark[16].x, handLandmarks.landmark[16].y]


    def calc_angle(self):
        m0 =  math.atan2(self.wrist[1] - self.thumb_cmc[1], (self.wrist[0] - self.thumb_cmc[0]))
        m1 =  math.atan2(self.thumb_mcp[1] - self.thumb_cmc[1], (self.thumb_mcp[0] - self.thumb_cmc[0]))
        m2 =  math.atan2(self.thumb_ip[1] - self.thumb_mcp[1], (self.thumb_ip[0] - self.thumb_mcp[0]))
        m3 =  math.atan2(self.thumb_tip[1] - self.thumb_ip[1], (self.thumb_tip[0] - self.thumb_ip[0]))

        m4 =  math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[0]))
        m5 =  math.atan2(self.index_pip[1] - self.index_mcp[1], (self.index_pip[0] - self.index_mcp[0]))
        m6 =  math.atan2(self.index_dip[1] - self.index_pip[1], (self.index_dip[0] - self.index_pip[0]))
        m7 =  math.atan2(self.index_tip[1] - self.index_dip[1], (self.index_tip[0] - self.index_dip[0]))

        m8 =  math.atan2(self.wrist[1] - self.middle_mcp[1], (self.wrist[0] - self.middle_mcp[0]))
        m9 =  math.atan2(self.middle_pip[1] - self.middle_mcp[1], (self.middle_pip[0] - self.middle_mcp[0]))
        m10 =  math.atan2(self.middle_dip[1] - self.middle_pip[1], (self.middle_dip[0] - self.middle_pip[0]))
        m11 =  math.atan2(self.middle_tip[1] - self.middle_dip[1], (self.middle_tip[0] - self.middle_dip[0]))

        m12 =  math.atan2(self.wrist[1] - self.ring_mcp[1], (self.wrist[0] - self.ring_mcp[0]))
        m13 =  math.atan2(self.ring_pip[1] - self.ring_mcp[1], (self.ring_pip[0] - self.ring_mcp[0]))
        m14 =  math.atan2(self.ring_dip[1] - self.ring_pip[1], (self.ring_dip[0] - self.ring_pip[0]))
        m15 =  math.atan2(self.ring_tip[1] - self.ring_dip[1], (self.ring_tip[0] - self.ring_dip[0]))

        m16 = math.atan2(self.wrist[1] - self.thumb_cmc[1], (self.wrist[0] - self.thumb_cmc[1]))
        m17 = math.atan2(self.wrist[1] - self.index_mcp[1], (self.wrist[0] - self.index_mcp[1]))


        self.thumb_wrist = math.atan(abs((m0 - m1) / (1 + m0 * m1))) - 0.8
        self.thumb_index = math.atan(abs((m16 - m17) / (1 + m16 * m17)))
        self.thumb_base = math.atan(abs((m1 - m2) / (1 + m1 * m2))) 
        self.thumb_knuckle = math.atan(abs((m2 - m3) / (1 + m2 * m3)))

        self.index_base = math.atan(abs((m4 - m5) / (1 + m4 * m5))) - 0.8
        self.index_knuckle = math.atan(abs((m5 - m6) / (1 + m5 * m6)))
        self.index_tip = math.atan(abs((m6 - m7) / (1 + m6 * m7))) 

        self.middle_base =  math.atan(abs((m8 - m9) / (1 + m8 * m9))) - 0.8
        self.middle_knuckle = math.atan(abs((m9 - m10) / (1 + m9 * m10)))
        self.middle_tip = math.atan(abs((m10 - m11) / (1 + m10 * m11))) 

        self.ring_base = math.atan(abs((m12 - m13) / (1 + m12 * m13))) - 0.8
        self.ring_knuckle = math.atan(abs((m13 - m14) / (1 + m13 * m14)))
        self.ring_tip = math.atan(abs((m14 - m15) / (1 + m14 * m15))) 


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
                if results.multi_hand_landmarks:
                    # since we will be dealing with only one hand
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.get_indices(hand_landmarks)
                        self.calc_angle()
                        # self.angles.data = [0.0,self.index_base, self.index_knuckle, self.index_tip,
                        # 0.0, self.middle_knuckle, self.middle_tip, self.middle_base,
                        # 0.0,  self.ring_knuckle,self.ring_tip,self.ring_base, 
                        # self.thumb_wrist, 0.0, self.thumb_base, self.thumb_knuckle,]
                        self.angles.data = [0.0, self.index_base, self.index_knuckle, self.index_tip,
                        0.0, self.middle_base, self.middle_knuckle, self.middle_tip,
                       0.0, self.ring_base, self.ring_knuckle, self.ring_tip, 
                        self.thumb_wrist, self.thumb_index, self.thumb_base, self.thumb_knuckle]
                        self.ang_pub.publish(self.angles)
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