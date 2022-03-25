#!/usr/bin/python
import rospy
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Header

def imu_cb(msg):
    pitch_angular_pub.publish(msg.angular_velocity.y)
    yaw_angular_pub.publish(msg.angular_velocity.z)
    euler = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    pitch_angle_pub.publish(euler[1])
    roll_angle_pub.publish(euler[0])
rospy.init_node("imu_data_unpacker")


pitch_angle_pub = rospy.Publisher("/balance/pitch_angle",Float64,queue_size=1)
roll_angle_pub = rospy.Publisher("/balance/roll_angle",Float64,queue_size=1)
pitch_angular_pub = rospy.Publisher("/balance/pitch_angular",Float64,queue_size=1)
yaw_angular_pub = rospy.Publisher("/balance/yaw_angular",Float64,queue_size=1)

imu_sub = rospy.Subscriber("/balance/imu_data",Imu,imu_cb,queue_size=1)

rospy.spin()
