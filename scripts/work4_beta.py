#!/usr/bin/env python
import smach, smach_ros, rospy, numpy, math, time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import util, detectshapes
from kobuki_msgs.msg import Led, Sound
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point, Quaternion
import cv2, cv_bridge
import tf


PARK_SPOT_WAYPOINTS = {'1': [Point(1.951, 1.130, 0.010), Quaternion(0.0, 0.0, -0.125, 0.992)],
                       '2': [Point(1.925, 0.324, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '3': [Point(1.854, -0.439, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '4': [Point(1.907, -1.249, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '5': [Point(1.975, -2.025, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '6': [Point(0.573, -0.054, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '7': [Point(0.586, -0.812, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '8': [Point(1.011, -2.088, 0.010), Quaternion(0.0, 0.0, -0.689, 0.725)]}

# Tianqi Wed version
PARK_SPOT_WAYPOINTS = {'1': [Point(1.951, 1.130, 0.010), Quaternion(0.0, 0.0, -0.125, 0.992)],
                       '2': [Point(1.925, 0.324, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '3': [Point(1.854, -0.439, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '4': [Point(1.907, -1.249, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '5': [Point(1.975, -2.025, 0.010), Quaternion(0.0, 0.0, 0.100, 0.995)],
                       '6': [Point(0.573, -0.054, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '7': [Point(0.586, -0.812, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '8': [Point(1.011, -2.088, 0.010), Quaternion(0.0, 0.0, -0.689, 0.725)]}

# Hum Thu version
PARK_SPOT_WAYPOINTS = {'1': [Point(1.970, 1.130, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '2': [Point(1.990, 0.424, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '3': [Point(1.964, -0.439, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '4': [Point(1.977, -1.249, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '5': [Point(1.985, -2.025, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '6': [Point(0.573, -0.004, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '7': [Point(0.586, -1.012, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '8': [Point(1.011, -2.088, 0.010), Quaternion(0.0, 0.0, -0.689, 0.725)]}

OFF_RAMP_WAYPOINT = [Point(0.067, -1.550, 0.010), Quaternion(0.0, 0.0, 0.195, 0.981)] #start

ON_RAMP_WAYPOINT = [Point(-0.310, 1.022, 0.010), Quaternion(0.0, 0.0, 0.994, -0.111)] #end

ROBOT_LENGTH = 0.365 #with bumper
BOX_EDGE_LENGTH = 0.334
SQUARE_DIST = 0.825


AMCL_APPROACH_BOX = 0.27

BOX_TAG_ID = 6
GOAL_TAG_ID = 30

BOX_SIDE_OFFSET_FORM_MIDDLE = 0.5
BOX_MIDDLE_OFFSET_FROM_TAG = (0, 0, -BOX_EDGE_LENGTH/2)
BOX_MIDDLE_ROTATION_FROM_TAG = (0, 0, 0, 1)

BOX_SIDES = {"box_front":[(0, 0, BOX_SIDE_OFFSET_FORM_MIDDLE ), (0, 0.707, 0, 0.707)],
             "box_left": [(-BOX_SIDE_OFFSET_FORM_MIDDLE, 0, 0), (0, 0, 0, 1)],
             "box_right":[(BOX_SIDE_OFFSET_FORM_MIDDLE, 0,  0), (0, 0, 1, 0)]}

class PushBox(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['completed', 'end', 'restart']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.twist_pub = None

        self.box_tag_id = None
        self.goal_tag_id = None
        #self.box_stall_id = None
        self.goal_stall_id = None

        self.box_tag_saw = None

        self.trial = 0

    def execute(self, userdata):
        if self.trial == 0:
            self.set_init_map_pose()
        self.trial += 1
        print "trial: ", self.trial
        if self.trial > 2:
            goal_pose = util.goal_pose('map', ON_RAMP_WAYPOINT[0], ON_RAMP_WAYPOINT[1]) 
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
            return 'completed'

        ar_tag_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_tag_sub_callback)
        print "Waiting for /ar_pose_marker message..."
        rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

        current_box_stall_id = None
        tmp_time = time.time()
        while True:
            if rospy.is_shutdown():
                ar_tag_sub.unregister()
                return 'end'
            if self.box_tag_id != None and self.goal_tag_id != None:
                try:
                    (trans, _) = self.listener.lookupTransform('/map', '/ar_marker_' + str(self.box_tag_id), rospy.Time(0))
                    box_tag_point = Point(trans[0], trans[1], trans[2])
                    current_box_stall_id= get_cloest_stall(box_tag_point)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print "TF look up exception when look up goal tag"
                

                if current_box_stall_id != None and self.goal_stall_id != None:
                    assert current_box_stall_id > 1 and current_box_stall_id < 5, \
                        "current_box_stall_id = {0}".format(str(current_box_stall_id))
                    assert current_box_stall_id != self.goal_stall_id, \
                        "current_box_stall_id = {0}, self.goal_stall_id = {1}".format(str(current_box_stall_id), \
                            str(self.goal_stall_id))
                    break
            twist = Twist()
            twist.angular.z = - 0.4
            self.twist_pub.publish(twist)

            if time.time() - tmp_time > 5:
                goal_pose = util.goal_pose('map', PARK_SPOT_WAYPOINTS['6'][0], Quaternion(0,0,0,1))
                self.client.send_goal(goal_pose)
                print "waiting for result ", goal_pose.target_pose.header.frame_id
                self.client.wait_for_result()
            
            if time.time() - tmp_time > 20:
                goal_pose = util.goal_pose('map', PARK_SPOT_WAYPOINTS['7'][0], Quaternion(0,0,0,1))
                self.client.send_goal(goal_pose)
                print "waiting for result ", goal_pose.target_pose.header.frame_id
                self.client.wait_for_result()

        box_is_left = True
        if int(current_box_stall_id) > int(self.goal_stall_id):
            box_is_left = False

        if box_is_left:
            point = PARK_SPOT_WAYPOINTS[str(current_box_stall_id-1)][0]
            quaternion = PARK_SPOT_WAYPOINTS[str(current_box_stall_id-1)][1]
            goal_pose = util.goal_pose('map', point, quaternion)
            self.client.send_goal(goal_pose)
            print "waiting for result ", goal_pose.target_pose.header.frame_id
            self.client.wait_for_result()
            #util.signal(1, onColor=Led.BLACK) #debug
            util.rotate(-92)
            for i in range(abs(current_box_stall_id - self.goal_stall_id)):
                if i == 0:
                    push_dist = SQUARE_DIST * 2 - 0.5
                    util.move(push_dist, linear_scale=0.1)
                else:
                    util.move(-0.6, linear_scale=0.3)
                    self.go_to_side('box_front')

                    push_dist = SQUARE_DIST + AMCL_APPROACH_BOX + 0.17
                    util.move(push_dist, linear_scale= 0.2)
        else:
            point = PARK_SPOT_WAYPOINTS[str(int(current_box_stall_id)+1)][0]
            quaternion = PARK_SPOT_WAYPOINTS[str(int(current_box_stall_id)+1)][1]
            goal_pose = util.goal_pose('map', point, quaternion)
            self.client.send_goal(goal_pose)
            print "waiting for result ", goal_pose.target_pose.header.frame_id
            self.client.wait_for_result()
            #util.signal(1, onColor=Led.BLACK) #debug
            util.rotate(90)
            for i in range(abs(current_box_stall_id - self.goal_stall_id)):
                if i == 0:
                    util.move(-0.2, linear_scale=0.3)
                else:
                    util.move(-0.6, linear_scale=0.3)
                self.go_to_side('box_front')
                if i == 0:
                    util.rotate(5)

                push_dist = SQUARE_DIST + AMCL_APPROACH_BOX + 0.07
                util.move(push_dist, linear_scale= 0.2)

        if self.fine_tune(box_is_left, push_dist) == 'lost_box':
            goal_pose = util.goal_pose('map', OFF_RAMP_WAYPOINT[0], OFF_RAMP_WAYPOINT[1])
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
            return 'restart'
        
        #util.move(0.3, linear_scale= 0.3)
        ar_tag_sub.unregister()
        return 'completed'
  
    def ar_tag_sub_callback(self, msg):
        for marker in msg.markers:
            if marker.id == BOX_TAG_ID:
                if self.box_tag_id == None:
                    self.box_tag_id = marker.id
                    util.signal(quantity=1, onColor=Led.RED)
                
                self.box_tag_saw = True
            else:
                self.box_tag_saw = False

            if marker.id == GOAL_TAG_ID:
                if self.goal_tag_id == None:
                    self.goal_tag_id = marker.id
                    util.signal(quantity=1, onColor=Led.GREEN)
                if self.goal_stall_id == None:
                    try:
                        (trans, _) = self.listener.lookupTransform('/map', '/ar_marker_' + str(self.goal_tag_id), rospy.Time(0))
                        goal_tag_point = Point(trans[0], trans[1], trans[2])
                        self.goal_stall_id = get_cloest_stall(goal_tag_point)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print "TF look up exception when look up goal tag"
    
    def go_to_side(self, side):
        while True:
            self.br.sendTransform(
                BOX_MIDDLE_OFFSET_FROM_TAG,
                BOX_MIDDLE_ROTATION_FROM_TAG,
                rospy.Time.now(),
                "box_middle",
                "ar_marker_" + str(BOX_TAG_ID),
            )
        
            self.br.sendTransform(
                BOX_SIDES[side][0],
                BOX_SIDES[side][1],
                rospy.Time.now(),
                side,
                "box_middle",
            )
            rospy.sleep(0.2)
            try:
                _, _ = self.listener.lookupTransform('/map', side, rospy.Time(0))
                goal_pose = util.goal_pose(side)
                self.client.send_goal(goal_pose)
                print "waiting for result ", goal_pose.target_pose.header.frame_id
                self.client.wait_for_result()
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "TF look up exception when look up goal tag"
            else:
                break
    
    def look_up_box(self, child_frame, parent_frame):
        self.br.sendTransform(
            BOX_MIDDLE_OFFSET_FROM_TAG,
            BOX_MIDDLE_ROTATION_FROM_TAG,
            rospy.Time.now(),
            "box_middle",
            "ar_marker_" + str(BOX_TAG_ID),
        )
    
        for key in BOX_SIDES:
            self.br.sendTransform(
                BOX_SIDES[key][0],
                BOX_SIDES[key][1],
                rospy.Time.now(),
                key,
                "box_middle",
            )
        rospy.sleep(0.2)
        while True:
            try:
                (trans, rots) = self.listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
                return Point(trans[0],trans[1],trans[2]), Quaternion(rots[0],rots[1],rots[2],rots[3])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "TF look up exception when look up goal tag"
    
    def fine_tune(self, box_is_left, ori_push_dist):
        if ori_push_dist > SQUARE_DIST * 2:
            ori_push_dist = SQUARE_DIST * 2

        util.move(-ori_push_dist, linear_scale = 0.3)

        tmp_time = time.time()
        while self.box_tag_saw == False:
            direction = 1 if box_is_left else -1
            twist = Twist()
            twist.angular.z = direction * 0.4
            self.twist_pub.publish(twist)
            if time.time() - tmp_time > 15:
                return 'lost_box'
        self.twist_pub.publish(Twist())
        
        box_middle_point, _ = self.look_up_box('box_middle', 'map')
        print box_middle_point, PARK_SPOT_WAYPOINTS[str(self.goal_stall_id)][0]
        error_x =  box_middle_point.x - PARK_SPOT_WAYPOINTS[str(self.goal_stall_id)][0].x
        if self.goal_stall_id == 1:
            error_x += 0.3
        error_y = box_middle_point.y - PARK_SPOT_WAYPOINTS[str(self.goal_stall_id)][0].y

        push_right_dist = error_y if error_y > 0 and abs(error_y) >= 0.5 else 0
        push_left_dist = -error_y if error_y < 0 and abs(error_y) >= 0.5 else 0
        push_forward = - error_x if error_x < 0 else 0

        if self.goal_stall_id == 5:
            push_left_dist = 0
        if self.goal_stall_id == 1:
            push_right_dist = 0

        if not box_is_left: #robot is right to the box
            print "fine_tune:", "push_right ", push_right_dist, "push_left", push_left_dist, "push_forward", push_forward
            if push_left_dist != 0:
                self.go_to_side('box_front')
                util.move(AMCL_APPROACH_BOX, linear_scale= 0.2)
                util.move(push_left_dist)
                util.move(-1, linear_scale=0.3)
            
            if push_forward != 0:
                self.go_to_side('box_left')
                util.move(AMCL_APPROACH_BOX, linear_scale=0.2)
                util.move(push_forward)
                util.move(-0.5, linear_scale= 0.3)
        
            if push_right_dist != 0:
                if push_forward == 0:
                    self.go_to_side('box_left')
                    util.move(-0.5, linear_scale= 0.3)
                self.go_to_side('box_left')
                util.move(AMCL_APPROACH_BOX, linear_scale=0.2)
                util.move(push_right_dist)
        else: # box is right to the goal, robot is left to the box
            print "fine_tune:", "push_right ", push_right_dist, "push_left", push_left_dist, "push_forward", push_forward
            if push_right_dist != 0:
                self.go_to_side('box_front')
                util.move(AMCL_APPROACH_BOX, linear_scale= 0.2)
                util.move(push_right_dist)
                util.move(-1, linear_scale=0.3)
            
            if push_forward != 0:
                self.go_to_side('box_right')
                util.move(AMCL_APPROACH_BOX, linear_scale=0.2)
                util.move(push_forward)
                util.move(-0.5, linear_scale= 0.3)

            if push_left_dist != 0:
                if push_forward == 0:
                    self.go_to_side('box_right')
                    util.move(-0.5, linear_scale= 0.3)
                self.go_to_side('box_right')
                util.move(AMCL_APPROACH_BOX, linear_scale=0.2)
                util.move(push_left_dist)
        
        util.signal(2, onColor=Led.GREEN)


    def set_init_map_pose(self):
        #referenced from https://www.cnblogs.com/kuangxionghui/p/8335853.html

        init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)
        rospy.loginfo("start set pose...")
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position = OFF_RAMP_WAYPOINT[0]
        p.pose.pose.orientation = OFF_RAMP_WAYPOINT[1]

        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

        init_pose_pub.publish(p)
        rospy.sleep(3)
            
class SearchContour(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['completed', 'end'],
                                input_keys=['SearchContour_in_contour'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.hsv = None

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else:
            contour = userdata.SearchContour_in_contour
            print "Looking for contour ", contour 

            cd = detectshapes.ContourDetector()
            image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
            print "Waiting for camera/rgb/image_raw message..."
            rospy.wait_for_message("camera/rgb/image_raw", Image)

            for stall_id in ['6', '7', '8']:
                goal_pose = util.goal_pose('map', 
                            point=PARK_SPOT_WAYPOINTS[stall_id][0],
                            quaternion=PARK_SPOT_WAYPOINTS[stall_id][1])
                self.client.send_goal(goal_pose)
                print "waiting for result ", goal_pose.target_pose.header.frame_id
                self.client.wait_for_result()
                while self.hsv == None:
                    pass
                _, red_contours = cd.getContours(self.hsv)
            
                if len(red_contours) > 0:
                    if red_contours[0] == contour:
                        util.signal(1, onColor=Led.ORANGE)
                        util.signal(2, onColor=Led.ORANGE)
                    return 'completed'
    
    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
class ON_RAMP(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['returned', 'end']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else:
            goal_pose = util.goal_pose('map', ON_RAMP_WAYPOINT[0], ON_RAMP_WAYPOINT[1])
            self.client.send_goal(goal_pose)
            print "waiting for result ", goal_pose.target_pose.header.frame_id
            self.client.wait_for_result()
            return 'returned'

def get_cloest_stall(target):
    minidist = float('inf')
    cloest_stall_id = 1
    for stall_id in PARK_SPOT_WAYPOINTS:
        stall_point = PARK_SPOT_WAYPOINTS[stall_id][0]
        dist = math.sqrt( (stall_point.x - target.x)**2 + (stall_point.y - target.y)**2 )
        #print stall_id, dist
        if dist < minidist:
            minidist = dist
            cloest_stall_id = stall_id
    return int(cloest_stall_id)

if __name__ == "__main__":
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end', 'returned'])
    sm.userdata.contour = detectshapes.Contour.Triangle

    with sm:
        smach.StateMachine.add('PushBox', PushBox(),
                                transitions={'completed':'SearchContour',
                                            'end':'end',
                                            'restart':'PushBox'
                                            })

        smach.StateMachine.add('SearchContour', SearchContour(),
                                transitions={'end':'end',
                                             'completed':'ON_RAMP'},
                                remapping={'SearchContour_in_contour':'contour'})

        smach.StateMachine.add('ON_RAMP', ON_RAMP(),
                                transitions={'end':'end',
                                             'returned':'returned'})

    outcome = sm.execute()
    rospy.spin()
