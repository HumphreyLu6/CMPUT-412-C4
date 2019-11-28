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


PARK_SPOT_WAYPOINTS = {'1': [Point(1.951, 1.130, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '2': [Point(1.925, 0.324, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '3': [Point(1.854, -0.439, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '4': [Point(1.907, -1.249, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '5': [Point(1.975, -2.025, 0.010), Quaternion(0.0, 0.0, 0.0, 1.0)],
                       '6': [Point(0.573, -0.054, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '7': [Point(1.011, -2.088, 0.010), Quaternion(0.0, 0.0, 1.0, 0.0)],
                       '8': [Point(-0.310, 1.022, 0.010), Quaternion(0.0, 0.0, -0.689, 0.725)]}

OFF_RAMP_WAYPOINT = [Point(0.067, -1.550, 0.010), Quaternion(0.0, 0.0, 0.195, 0.981)] #start

ON_RAMP_WAYPOINT = [Point(-0.310, 1.022, 0.010), Quaternion(0.0, 0.0, 0.994, -0.111)] #end

class PushBox(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['completed', 'end']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.listener = tf.TransformListener()
        self.box_tag_id = None
        self.goal_tag_id = None
        self.box_waypoint = None
        self.goal_waypoint = None

    def execute(self, userdata):
        self.set_init_map_pose()

        ar_tag_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_tag_sub_callback)
        print "Waiting for /ar_pose_marker message..."
        rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
        twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

        tmp_time = time.time()
        while True:
            if rospy.is_shutdown():
                return 'end'
            if self.box_tag_id != None and self.goal_tag_id != None:
                break
            twist = Twist()
            twist.angular.z = - 0.2
            twist_pub.publish(twist)

            if time.time() - tmp_time > 5:
                goal_pose = util.goal_pose('map', PARK_SPOT_WAYPOINTS['6'][0], PARK_SPOT_WAYPOINTS['6'][1])
                self.client.send_goal(goal_pose)
                self.client.wait_for_result()
                util.rotate(87)
            
        while True:
            if rospy.is_shutdown():
                ar_tag_sub.unregister()
                return 'end'
            try:
                (trans,rots) = self.listener.lookupTransform('map', '/ar_marker_' + str(self.box_tag_id), rospy.Time(0))
                point = Point(trans[0], trans[1], trans[2])
                #quaternion = Quaternion(rots[0],rots[1],rots[2],rots[3])
                quaternion = Quaternion(0, 0, 0.118, 0.993)
                self.box_waypoint = util.goal_pose('map', point, quaternion)
                print "self.box_waypoint", self.box_waypoint
                (trans,rots) = self.listener.lookupTransform('/map', '/ar_marker_' + str(self.goal_tag_id), rospy.Time(0))
                point = Point(trans[0], trans[1], 0.010)
                # quaternion = Quaternion(rots[0],rots[1],rots[2],rots[3])
                quaternion = Quaternion(0, 0, 0.118, 0.993)
                self.goal_waypoint = util.goal_pose('map', point, quaternion)
                print "self.goal_waypoint", self.goal_waypoint
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            finally:
                if self.box_waypoint != None and self.goal_waypoint != None:
                    break
                print "Try to get waypoints:", self.box_waypoint, self.goal_waypoint
        
        
        print get_cloest_stall(self.goal_waypoint.target_pose.pose.position)

        assert self.box_waypoint != None and self.goal_waypoint != None

        self.box_stall_id = get_cloest_stall(self.box_waypoint.target_pose.pose.position)
        self.goal_stall_id = get_cloest_stall(self.goal_waypoint.target_pose.pose.position)
        print self.box_stall_id, self.goal_stall_id
        assert self.box_stall_id != self.goal_stall_id

        box_is_left = True
        if int(self.box_stall_id) > int(self.goal_stall_id):
            box_is_left = False

        square_dist = 0.825
        if box_is_left:
            point = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)-1)][0]
            quaternion = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)-1)][1]
            goal_pose = util.goal_pose('map', point, quaternion)
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
            util.signal(1, onColor=Led.BLACK) #debug
            util.rotate(-87)
            push_dist = (abs( int(self.box_stall_id)- int(self.goal_stall_id)) + 1) * square_dist - 0.4
            util.move(push_dist)
            util.signal(2, onColor=Led.GREEN)
            util.rotate(87)
        else:
            point = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)+1)][0]
            quaternion = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)+1)][1]
            goal_pose = util.goal_pose('map', point, quaternion)
            self.client.send_goal(goal_pose)
            self.client.wait_for_result()
            util.signal(1, onColor=Led.BLACK) #debug
            util.rotate(87)
            push_dist = abs( int(self.box_stall_id)- int(self.goal_stall_id) + 1) * square_dist
            util.move(push_dist)
            util.signal(2, onColor=Led.GREEN)
            util.rotate(-87)

        ar_tag_sub.unregister()
        return 'completed'
  
    def ar_tag_sub_callback(self, msg):
        for marker in msg.markers:
            if marker.id == 6 and self.box_tag_id == None:
                self.box_tag_id = marker.id
                util.signal(quantity=1, onColor=Led.RED)
            if marker.id == 30 and self.goal_tag_id == None:
                self.goal_tag_id = marker.id
                util.signal(quantity=1, onColor=Led.GREEN)
    
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
            assert isinstance(contour, detectshapes.Contour)

            cd = detectshapes.ContourDetector()
            image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
            print "Waiting for camera/rgb/image_raw message..."
            rospy.wait_for_message("camera/rgb/image_raw", Image)

            for stall_id in ['5', '6', '7']:
                goal_pose = util.goal_pose('map', 
                            point=PARK_SPOT_WAYPOINTS[stall_id][0],
                            quaternion=PARK_SPOT_WAYPOINTS[stall_id][1])
                self.client.send_goal(goal_pose)
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
            goal = util.goal_pose('map', ON_RAMP_WAYPOINT[0], ON_RAMP_WAYPOINT[1])
            self.client.send_goal(goal)
            self.client.wait_for_result()

            self.move_forward(0.2)
            return 'returned'

    def move_forward(self, meters):
        twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
        tmp = time.time()
        while (time.time() - tmp) < 2:
            twist = Twist()
            twist.linear.x = 0.3
            twist_pub.publish(twist)

def get_cloest_stall(target):
    minidist = float('inf')
    cloest_stall_id = 1
    for stall_id in PARK_SPOT_WAYPOINTS:
        stall_point = PARK_SPOT_WAYPOINTS[stall_id][0]
        dist = math.sqrt( (stall_point.x - target.x)**2 + (stall_point.y - target.y)**2 )
        print stall_id, dist
        if dist < minidist:
            minidist = dist
            cloest_stall_id = stall_id
    return cloest_stall_id

if __name__ == "__main__":
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end', 'returned'])
    sm.userdata.contour = detectshapes.Contour.Triangle

    with sm:
        smach.StateMachine.add('PushBox', PushBox(),
                                transitions={'completed':'SearchContour',
                                            'end':'end'
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
