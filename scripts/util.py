#!/usr/bin/env python
import rospy
import numpy
from enum import Enum
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from rospy import ROSException
from ros_numpy import numpify
from tf.transformations import decompose_matrix
from kobuki_msgs.msg import Led, Sound
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

def approxEqual(a, b, tol = 0.001):
    return abs(a - b) <= tol

def signal(quantity=1,onColor=Led.GREEN,offColor=Led.BLACK,interval=0.5,onColor2=None):
    
    if quantity == 1:
        led_pub_1.publish(onColor)
    elif quantity == 2:
        led_pub_1.publish(onColor)
        if onColor2 != None:
            led_pub_2.publish(onColor2)
        else:
            led_pub_2.publish(onColor)
    else:
        led_pub_1.publish(onColor)
        led_pub_2.publish(onColor)

    for i in range(quantity):
        sound_pub.publish(1)
        rospy.sleep(interval)

    led_pub_1.publish(offColor)
    led_pub_2.publish(offColor)

def wait_for_odom_angle(timeout=None):
    odom = rospy.wait_for_message("odom", Odometry, timeout=timeout)
    pose = numpify(odom.pose.pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    theta = angles[2] * 180 / 3.14159
    return theta

def goal_pose(frame_id, point = Point(0, 0, 0), quaternion = Quaternion(0, 0, 0, 1)):  
    '''
    Params: frame_id, point = Point(0, 0, 0), quaternion = Quaternion(0, 0, 0, 1)
    Return: goal_pose
    '''  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = frame_id
    goal_pose.target_pose.pose.position = point
    goal_pose.target_pose.pose.orientation = quaternion
    return goal_pose
    
def rotate(angle=90, max_error=3, anglular_scale=1.0):
    '''
    input: angle=90, max_error=3, anglular_scale=1.0, positive numbers turn left
    '''

    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = numpy.sign(angle)
    target_theta = init_theta + angle
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360

    while abs(target_theta - theta) > max_error:
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()
    twist_pub.publish(Twist())

def move(distance=0.1, max_error=0.03, linear_scale=0.1):
    '''
    params: distance=0.1, max_error=0.03, linear_scale=0.1
    '''
    odom = rospy.wait_for_message("odom", Odometry)
    init_pose = odom
    while (init_pose.pose.pose.position.x - odom.pose.pose.position.x)**2 + (init_pose.pose.pose.position.y - odom.pose.pose.position.y)**2 < abs(distance)**2:
        out_twist = Twist()
        out_twist.linear.x = numpy.sign(distance) * linear_scale
        twist_pub.publish(out_twist)
        odom = rospy.wait_for_message("odom", Odometry)
    twist_pub.publish(Twist())

if __name__ == "__main__":
    rospy.init_node("Util_Test")
    while not rospy.is_shutdown():
        rotate(90,anglular_scale=1)
        signal(1, onColor=Led.ORANGE)
        rotate(-180,anglular_scale=2)
        signal(2,onColor=Led.RED)
        rotate(180, anglular_scale=2)
        signal(3,interval=1)
        quit()