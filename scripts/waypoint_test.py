#!/usr/bin/env python
# BEGIN ALL
import rospy, numpy, actionlib
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import smach
import smach_ros


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['navigating', 'wait', 'end'])
    
    def execute(self, userdata):
        global g_targets, g_start
        if rospy.is_shutdown():
            return 'end'

        if g_start == True:
            if len(g_targets) > 0:
                return 'navigating'
            else:
                print 'No targets, can not start.'
                return 'wait'
        else:
            return 'wait'

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['navigating', 'arrived', 'end'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        global g_start
        if rospy.is_shutdown():
            return 'end'
        else:
            waypoints = self.get_waypoints()
            for pose in waypoints:   
                goal = self.goal_pose(pose)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                rospy.sleep(1.5)
            g_start = False
            return 'arrived'
    
    def goal_pose(self, pose): 
        goal_pose = MoveBaseGoal()

        goal_pose.target_pose.header.frame_id = 'map'

        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]

        return goal_pose
    
    def get_waypoints(self):
        waypoints = [#((-1.895, -0.507, 0.010), [0.0, 0.0, 0.156, 0.988]), #on ramp
                     #((-0.822, -0.009, 0.010), [0.0, 0.0, 0.118, 0.993]), #search1
                     #((-1.313, 1.434, 0.010), [0.0, 0.0, 0.127, 0.992]),  #search2
                     ((-0.803, 2.353, 0.010), [0.0, 0.0, 0.105, 0.994]),  #park1  
                     ((-0.608, 1.657, 0.010), [0.0, 0.0, 0.134, 0.991]),  #park2
                     ((-0.401, 0.869, 0.010), [0.0, 0.0, 0.124, 0.992]),  #park3
                     ((-0.291, 0.079, 0.010), [0.0, 0.0, 0.106, 0.994]),  #park4
                     ((-0.169, -0.719, 0.010), [0.0, 0.0, 0.092, 0.996]), #park5
                     ((-1.922, 1.029, 0.010), [0.0, 0.0, 0.997, -0.080]), #park6
                     ((-1.694, 0.291, 0.010), [0.0, 0.0, 0.992, -0.123]), #park7
                     ((-1.077, -0.846, 0.010), [0.0, 0.0, -0.621, 0.784]),#park8
                     ((-2.901, 1.809, 0.010), [0.0, 0.0, 0.960, -0.281])] #off ramp
        return waypoints
            
def joy_callback(msg):
    global g_targets, g_start
    if len(g_targets) < 4:
        if msg.buttons[0] == 1:
            g_targets.append(1)
        if msg.buttons[1] == 1:
            g_targets.append(2)
        if msg.buttons[2] == 1:
            g_targets.append(3)
        if msg.buttons[3] == 1:
            g_targets.append(4)
    else:
        print "targets full !"      

    if msg.buttons[5] == 1:
        g_start = not g_start  
    
    print g_targets

def main():
    global g_targets, g_start
    g_targets = []
    g_start = False

    rospy.init_node("navi_bot")

    rospy.Subscriber("joy", Joy, joy_callback)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        
        smach.StateMachine.add('Wait', Wait(),
                                transitions={'navigating':'Navigate',
                                             'end':'end',
                                             'wait':'Wait'})
        smach.StateMachine.add('Navigate', Navigate(),
                                transitions={'arrived':'Wait',
                                             'navigating':'Navigate'})
    outcome = sm.execute()

    rospy.spin()

if __name__ == "__main__":
    g_targets = None
    g_start = None
    main()

        
