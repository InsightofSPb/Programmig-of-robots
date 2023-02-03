import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import Image


Kz = 5.0
Bz = 3.0
z_des = 2.5

Kw = 0.045
Bw = 0.001

Ky = 0.006
By = 0.002

class Contoller:

    def __init__(self):
        rospy.init_node("controller_node")

        rospy.on_shutdown(self.stop_robot)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)
        rospy.Subscriber("/cam_1/camera/image", Image, self.image_callback)
        self.position = Point()
        self.twist = Twist()
        self.bridge = CvBridge()
        self.omega_error = 0
        self.omega_error_prev = 0
        self.y_error = 0
        self.y_error_prev = 0

    def __del__(self):
        self.stop_robot

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_image, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(cv_image, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(cv_image, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(cv_image, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy
            
            cv2.circle(cv_image, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(cv_image, (160, 120), (cy, int(msg.height/2)), (0, 255, 0), 3)
        cv2.imshow("Image window2", cv_image)
        cv2.waitKey(3)  

    def enable_motors(self):
        rospy.wait_for_service('/enable_motors')
        try:
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(True)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def stop_robot(self):
        # cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # cmd_pub.publish(Twist())
        self.cmd_pub.publish(Twist())
        # rospy.sleep(1)

    def spin(self):
        self.enable_motors()

        try:
            rate = rospy.Rate(50)
            while not rospy.is_shutdown():

                uz = Kz * (z_des - self.position.z) - Bz* self.twist.angular.z
                
                uw = Kw * self.omega_error - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)

                self.omega_error_prev = self.omega_error

                uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 50.0)
                self.y_error_prev = self.y_error
                
                cmd_msg = Twist()

                cmd_msg.linear.x = 1.5
                cmd_msg.linear.y = uy
                cmd_msg.linear.z = uz
                cmd_msg.angular.z = -uw

                self.cmd_pub.publish(cmd_msg)
                rate.sleep()
        except:
            self.stop_robot()



def main():
    ctrl = Contoller()
    ctrl.spin()

if __name__=="__main__":
    main()