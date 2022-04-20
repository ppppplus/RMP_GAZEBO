#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
def calculate_mini_4wd_link_velocity_steering(data):

    pub_vel_mini_4wd_base_footprint_velocity=rospy.Publisher('mini_4wd_base_footprint_velocity/link_cmd_vel', Float64,queue_size=1)
    pub_vel_mini_4wd_base_footprint_steering=rospy.Publisher('mini_4wd_base_footprint_steering/link_cmd_vel', Float64,queue_size=1)
    v=data.twist
    if(len(v) == 16):     
        wbase=v[11].angular.z
        vbase=v[11].linear.x

        pub_vel_mini_4wd_base_footprint_velocity.publish(vbase)
        pub_vel_mini_4wd_base_footprint_steering.publish(wbase)
def calculate_mini_4wd_joint_velocity_steering(data):

    pub_vel_mini_4wd_left_front_wheel_joint = rospy.Publisher('mini_4wd_left_front_wheel_velocity/joint_cmd_vel', Float64, queue_size=1)
    pub_vel_mini_4wd_right_front_wheel_joint = rospy.Publisher('mini_4wd_right_front_wheel_velocity/joint_cmd_vel', Float64, queue_size=1)
    pub_vel_mini_4wd_left_rear_wheel_joint = rospy.Publisher('mini_4wd_left_rear_wheel_velocity/joint_cmd_vel', Float64, queue_size=1)
    pub_vel_mini_4wd_right_rear_wheel_joint = rospy.Publisher('mini_4wd_right_rear_wheel_velocity/joint_cmd_vel', Float64, queue_size=1)
    a=data.velocity
    #rospy.loginfo(a)
    left_front_wheel_joint_velocity=a[0]*0.04  #v=wr
    right_front_wheel_joint_velocity=a[2]*0.04
    left_rear_wheel_joint_velocity=a[1]*0.04
    right_rear_wheel_joint_velocity=a[3]*0.04
    
    pub_vel_mini_4wd_left_front_wheel_joint.publish(left_front_wheel_joint_velocity)
    pub_vel_mini_4wd_right_front_wheel_joint.publish(right_front_wheel_joint_velocity)
    pub_vel_mini_4wd_left_rear_wheel_joint.publish(left_rear_wheel_joint_velocity)
    pub_vel_mini_4wd_right_rear_wheel_joint.publish(right_rear_wheel_joint_velocity)

def Pub_mini_4wd_real_cmd_vel():

    rospy.init_node('real_cmd_vel_mini_4wd', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, calculate_mini_4wd_link_velocity_steering)
    rospy.Subscriber("joint_states", JointState, calculate_mini_4wd_joint_velocity_steering)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Pub_mini_4wd_real_cmd_vel()
    except rospy.ROSInterruptException:
        pass
