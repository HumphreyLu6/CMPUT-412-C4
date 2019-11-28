#!/usr/bin/env python
# BEGIN ALL
import rospy, numpy, actionlib
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import util
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
                util.signal()
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
        # waypoints = [((0.067, -1.550, 0.010), [0.0, 0.0, 0.195, 0.981]), #off ramp
        #              ((1.951, 1.130, 0.010), [0.0, 0.0, 0, 1]),  #park1  
        #              ((1.925, 0.324, 0.010), [0.0, 0.0, 0, 1]),  #park2
        #              ((1.854, -0.439, 0.010), [0.0, 0.0, 0, 1]),  #park3
        #              ((1.907, -1.249, 0.010), [0.0, 0.0, 0, 1]),  #park4
        #              ((1.975, -2.025, 0.010), [0.0, 0.0, 0, 1]), #park5
        #              ((0.573, -0.054, 0.010), [0.0, 0.0, 1.0, 0.0]), #park6
        #              ((0.586, -0.812, 0.010), [0.0, 0.0, 1.0, 0.0]), #park7
        #              ((1.011, -2.088, 0.010), [0.0, 0.0, -0.689, 0.725]),#park8
        #              ((-0.310, 1.022, 0.010), [0.0, 0.0, 0.994, -0.111])] #off ramp
        waypoints = [((0.067, -1.550, 0.010), [0.0, 0.0, 0.195, 0.981]), #on ramp
                     #((-0.822, -0.009, 0.010), [0.0, 0.0, 0.118, 0.993]), #search1
                     #((-1.313, 1.434, 0.010), [0.0, 0.0, 0.127, 0.992]),  #search2
                     ((1.951, 1.130, 0.010), [0.0, 0.0, -0.125, 0.992]),  #park1  
                     ((1.925, 0.324, 0.010), [0.0, 0.0, 0.100, 0.995]),  #park2
                     ((1.854, -0.439, 0.010), [0.0, 0.0, 0.100, 0.995]),  #park3
                     ((1.907, -1.249, 0.010), [0.0, 0.0, 0.100, 0.995]),  #park4
                     ((1.975, -2.025, 0.010), [0.0, 0.0, 0.100, 0.995]), #park5
                     ((0.573, -0.054, 0.010), [0.0, 0.0, 1.0, 0.0]), #park6
                     ((0.586, -0.812, 0.010), [0.0, 0.0, 1.0, 0.0]), #park7
                     ((1.011, -2.088, 0.010), [0.0, 0.0, -0.689, 0.725]),#park8
                     ((-0.310, 1.022, 0.010), [0.0, 0.0, 0.994, -0.111])] #off ramp
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
    
    print g_targets, g_start
    print msg.buttons

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

        
