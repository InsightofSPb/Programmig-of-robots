import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import Image

alt = 2.5
Kz = 2.0
Bz = 3.0


Kw = 0.02
Bw = 0.001

Ky = 0.01
By = 0.001

RING_AVOIDANCE_TIME = 5 # [seconds]

class Trajectory_control():

    def __init__(self):
        rospy.init_node("controller_node")

        rospy.on_shutdown(self.stop_robot)

        self.enable_motors(True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)
        rospy.Subscriber("/cam_1/camera/image", Image, self.camera_bottom_callback)
        rospy.Subscriber("/cam_2/camera/image", Image, self.camera_front_callback)
        
        self.position = Point()
        self.twist = Twist()
        self.bridge = CvBridge()
        self.omega_error = 0
        self.omega_error_prev = 0
        self.y_error = 0
        self.y_error_prev = 0
        self.z_des = alt
        self.state = "free_flight"
        self.red_ring_detected = False
        self.blue_ring_detected = False
        self.time_start_up = 0
        self.avoidance_time = 0
        self.e_x_blue, self.e_y_blue = 0, 0
        self.cam_bottom = []
        self.cam_front = []
        
        self.rate=rospy.Rate(30)

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def stop_robot(self):
        self.enable_motors(False)

    def camera_bottom_callback(self, msg):
        try:
            self.cam_bottom = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        grey_image = cv2.cvtColor(self.cam_bottom, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        self.cam_bottom = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(self.cam_bottom, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(self.cam_bottom, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(self.cam_bottom, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(self.cam_bottom, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(self.cam_bottom, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy
            
            cv2.circle(self.cam_bottom, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(self.cam_bottom, (160, 120), (cy, int(msg.height/2)), (0, 255, 0), 3)
 

    def camera_front_callback(self, msg):
       # """ Computer vision stuff for Rings"""
       try:
            self.cam_front = self.bridge.imgmsg_to_cv2(msg, "bgr8")
       except CvBridgeError as e:
           rospy.logerr("CvBridge Error: {0}".format(e))


       # red
       lower = np.uint8([0, 0, 90])
       upper = np.uint8([30, 30, 120])
       self.cam_front, red_pose, red_radius  = self.ring_detector(self.cam_front, lower, upper, (0,0,255))


       # blue
       lower = np.uint8([40, 20, 20])
       upper = np.uint8([80, 50, 50])
       self.cam_front, blue_pose, blue_radius = self.ring_detector(self.cam_front, lower, upper, (255,0,0))


       # print(red_radius, blue_radius)


       if 50 < red_radius < 70 or 50 < blue_radius < 80:
           if red_radius > blue_radius:
               self.blue_ring_detected = False
               self.red_ring_detected = True
           else:
               self.red_ring_detected = False
               self.blue_ring_detected = True
              
               # offset in ring xy-plane to fly through center of a ring
               # error = <center of image> - <center of ring>
               self.e_x_blue = 160 - blue_pose[0]
               self.e_y_blue = 120 - blue_pose[1]
       else:
           self.blue_ring_detected = False
           self.red_ring_detected = False


    def images(self, img, title):
        cv2.imshow(title, img)
        cv2.waitKey(3)    


    def enable_motors(self, state):
        try:
            rospy.wait_for_service('/enable_motors', 2)
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(state)
            print('Motors started')
        except rospy.ServiceException as e:
            print("Cannot start motors: ", e)


    def ring_detector(self, image, lower, upper, color):
       color_mask = cv2.inRange(image, lower, upper)
       color_contours, _ = cv2.findContours(color_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
       if color_contours:
           max_len_c = 0
           c = color_contours[0]
           for i in range(0, len(color_contours)):
               if len(color_contours[i]) > max_len_c:
                   c = color_contours[i]
                   max_len_c = len(color_contours[i])
           self.color_distance = max_len_c
           M = cv2.moments(c)
           if M['m00'] != 0:
               cx = int(M['m10']/M['m00'])
               cy = int(M['m01']/M['m00'])
           else:
               cx = 0
               cy = 0
           (x1,y1), color_r = cv2.minEnclosingCircle(c)
           if color_r > 10:
               image = cv2.circle(image, (cx, cy), radius=5, color=color, thickness=-1)
               cv2.drawContours(color_r, c, -1, (0,255,0), 1)
               color_r = cv2.circle(color_r, (int(x1), int(y1)), radius=int(color_r), color=color, thickness=4)       
               return image, (x1,y1), color_r[0]
       return image, (0,0), 0

    def fsm_update(self):
        if self.red_ring_detected:
           self.state = "drone_up"
           print('Red ring detected')
        elif 0 <  self.avoidance_time <= RING_AVOIDANCE_TIME:
           self.state = "drone_up"
           print('Going up')
        elif self.blue_ring_detected:
           self.state = "drone_blue_ring"
           print('Blue ring detected')
        else:
           self.state = "free_flight"
           print('Free flight')
           self.time_start_up = 0
           self.avoidance_time = 0



    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.fsm_update()
                if self.state == "drone_up":
                    self.z_des = 5
                    if self.time_start_up == 0:
                        self.time_start_up = rospy.get_time()
                    self.avoidance_time = rospy.get_time() - self.time_start_up 
                elif self.state == "drone_blue_ring":
                    self.z_des += 0.001 * self.e_y_blue
                    
                elif self.state == "free_flight":
                    self.z_des = alt
                else:
                    rospy.logerr("Error: state name error!")

                print(self.avoidance_time)
           
                uz = Kz * (self.z_des - self.position.z) - Bz* self.twist.linear.z
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

                if len(self.cam_bottom) > 0 and len(self.cam_front) > 0:
                    self.images(self.cam_bottom, title='Bottom')
                    self.images(self.cam_front, title='Front')

            except KeyboardInterrupt:
                break
            self.rate.sleep()




def main():
    ctrl = Trajectory_control()
    ctrl.spin()

if __name__=="__main__":
    main()
