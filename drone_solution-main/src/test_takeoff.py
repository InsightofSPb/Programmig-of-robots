import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

K = 1.0
B = 2.0
z_des = 5.0


def callback(msg):

    z = msg.pose.pose.position.z
    dotz = msg.twist.twist.linear.z

    uz = K * (z_des - z) - B * dotz
    

   
    cmd_msg = Twist()
    cmd_msg.linear.z = uz


    cmd_pub.publish(cmd_msg)

def enable_motors():
    rospy.wait_for_service('/enable_motors')
    try:
        call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
        resp1 = call_em(True)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node("test_takeoff_node")

    if enable_motors():
        print("Motors started!")
    else:
        print("Motors didn't start")    

    rospy.Subscriber("/ground_truth/state", Odometry, callback)

    rospy.spin()
    

if __name__ =="__main__":
    main()