import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import Image

import HandDetection as htm
import numpy as np

bridge = CvBridge()

Kz = 0.5
Bz = 0.5


Kw = 1
Bw = 1

Kx = 1
Bx = 0.3



class Contoller:

    def __init__(self):
        rospy.init_node("controller_node")
        self.enable_motors()
        
        
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)
        
        self.position = Point()
        self.twist = Twist()
        
        rospy.Subscriber("/cam_2/camera/image", Image, self.camera_front_callback)        
        self.bridge = CvBridge()
        self.omega_error = 0
        self.omega_error_prev = 0

        self.x_error = 0
        self.x_error_prev = 0
        self.cam_front = []
        self.rate=rospy.Rate(50)
        
        

        

        self.detector = htm.handDetector()
        self.cap = cv2.VideoCapture(0)
        wCam, hCam = 640, 480
        self.cap.set(3, wCam)
        self.cap.set(4, hCam)

    def enable_motors(self):
        try:
            rospy.wait_for_service('/enable_motors')
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(True)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def camera_front_callback(self, msg):
        try:
            self.cam_front = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def images(self, img, title):
        cv2.imshow(title, img)
        cv2.waitKey(3)   
        
    def spin(self):

        while not rospy.is_shutdown():

            try:            
                success, img = self.cap.read()
                img = self.detector.findHands(img)
                lmList = self.detector.findPosition(img)
                z_des = self.detector.findDistanceAlt(img)
                x_des = self.detector.findDistanceX(img)
                wz_des = self.detector.findDistanceWZ(img)
   
                      
                uz = Kz * (z_des - self.position.z) - Bz * self.twist.linear.z
                uwz = Kw * (wz_des - self.omega_error) - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                self.omega_error_prev = self.omega_error
                
                ux = Kx * (x_des - self.x_error) - Bx * (self.x_error - self.x_error_prev) / (1.0 / 50)
                self.x_error_prev = self.x_error
                
                cmd_msg = Twist()

                cmd_msg.linear.x = ux
                cmd_msg.linear.y = 0
                cmd_msg.linear.z = uz
                cmd_msg.angular.z = -uwz
                
                print(f'Желаемая высота: {z_des}, Скорость по Х: {ux}, Скорость поворота: {uwz}')
                
                self.cmd_pub.publish(cmd_msg)                
                
                cv2.imshow('Hand', img)
                cv2.waitKey(3)  
                
                if len(self.cam_front > 0):
                    self.images(self.cam_front, title='Front')
                
                self.rate.sleep()



            except KeyboardInterrupt:
                break




def main():
    ctrl = Contoller()
    ctrl.spin()


if __name__=='__main__':
    main()
