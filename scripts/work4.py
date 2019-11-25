#!/usr/bin/env python
import smach, smach_ros, rospy, numpy, math, time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import util, detectshapes
from kobuki_msgs.msg import Led, Sound
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import cv2, cv_bridge

SEARCH_WAYPOINTS = {'1': [(-0.822, -0.009, 0.010), (0, 0, 0.118, 0.993)],
                    '2': [(-1.313,  1.434, 0.010), (0, 0, 0.127, 0.992)]}

PARK_SPOT_WAYPOINTS = {'1': [(-0.803,  2.353, 0.010), (0, 0,  0.105,  0.994)],
                       '2': [(-0.608,  1.657, 0.010), (0, 0,  0.134,  0.991)],
                       '3': [(-0.401,  0.869, 0.010), (0, 0,  0.124,  0.992)],
                       '4': [(-0.291,  0.079, 0.010), (0, 0,  0.106,  0.994)],
                       '5': [(-0.169, -0.719, 0.010), (0, 0,  0.092,  0.996)],
                       '6': [(-1.922,  1.029, 0.010), (0, 0,  0.997, -0.080)],
                       '7': [(-1.694,  0.291, 0.010), (0, 0,  0.992, -0.123)],
                       '8': [(-1.077, -0.846, 0.010), (0, 0, -0.621,  0.784)]}

OFF_RAMP_WAYPOINT = [(-1.895, -0.507, 0.010), (0.000, 0.000, 0.326, 0.945)] #start


ON_RAMP_WAYPOINT = [(-2.901, 1.809, 0.010), (0.000, 0.000, 0.960, -0.281)] #end
#ON_RAMP_WAYPOINT = [(-2.961, 1.680, 0.010), (0.000, 0.000, 0.960, -0.281)] #end

class Park(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['next', 'end', 'return'],
                                input_keys=['Park_in_process'],
                                output_keys=['Park_in_process']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else:
            process = userdata.Park_in_process
            if process['spot_id'] == 1:
                self.set_init_map_pose()
            if process['spot_id'] > 8:
                return 'return'
            waypoint = PARK_SPOT_WAYPOINTS[str(process['spot_id'])]
            goal = util.goal_pose(waypoint, 'map', 'list')
            self.client.send_goal(goal)
            self.client.wait_for_result()
            #util.signal(process['spot_id'], onColor=Led.BLACK) #debug
            search_orientation = [0]
            if process['spot_id'] == 1:
                search_orientation.append(90)
            elif process['spot_id'] == 5:
                search_orientation.append(-90)

            process = self.search(process, search_orientation)
            process['spot_id'] += 1
            if process['ARtag_found'] and process['contour_found'][1] and process['unmarked_spot_id'][1]:
                return 'return'
            return 'next'


    def search(self, process, search_orientation = [0]):
        if process['ARtag_found'] == False:
            ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_callback)
            rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
        if process['contour_found'][1] == False:
            image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
            rospy.wait_for_message("camera/rgb/image_raw", Image)

        for angle in search_orientation:
            if angle != 0:
                util.rotate(angle)
            if process['ARtag_found'] == False and self.search_ARtag():
                util.signal(1, onColor=Led.GREEN)
                process['ARtag_found'] = True

            if process['contour_found'][1] == False and self.search_contour(process):
                util.signal(1, onColor=Led.ORANGE)
                process['contour_found'][1] = True

            if process['ARtag_found'] and process['contour_found'][1]:
                break

        #unregister

        if process['spot_id'] == process['unmarked_spot_id'][0]:
            util.signal(1, onColor=Led.RED)
            process['unmarked_spot_id'][1] = True
        return process

    def search_ARtag(self):
        if len(self.tags) != 0:
            print "tag_id:", self.tags[0]
            return True
        return False

    def search_contour(self, process):
        cd = detectshapes.ContourDetector()
        _, red_contours = cd.getContours(self.hsv)
        if len(red_contours) > 0:
            if red_contours[0] == process['contour_found'][0]:
                return True
        return False

    def ar_callback(self, msg):
        self.tags = []
        for marker in msg.markers:
            self.tags.append(int(marker.id))

    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def set_init_map_pose(self):
        #referenced from https://www.cnblogs.com/kuangxionghui/p/8335853.html

        init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)
        rospy.loginfo("start set pose...")
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position.x = OFF_RAMP_WAYPOINT[0][0]
        p.pose.pose.position.y = OFF_RAMP_WAYPOINT[0][1]
        p.pose.pose.position.z = OFF_RAMP_WAYPOINT[0][2]

        p.pose.pose.orientation.x = OFF_RAMP_WAYPOINT[1][0]
        p.pose.pose.orientation.y = OFF_RAMP_WAYPOINT[1][1]
        p.pose.pose.orientation.z = OFF_RAMP_WAYPOINT[1][2]
        p.pose.pose.orientation.w = OFF_RAMP_WAYPOINT[1][3]

        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

        for _ in range(1):
            tmp = time.time()
            init_pose_pub.publish(p)
            #util.signal(1)
        rospy.sleep(3)

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
            goal = util.goal_pose(ON_RAMP_WAYPOINT,frame_id='map')
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

if __name__ == "__main__":
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.process = {'spot_id': 1,
                            'ARtag_found': False,
                            'contour_found': [None,False],
                            'unmarked_spot_id': [8,False]
                            }

    with sm:
        smach.StateMachine.add('Park', Park(),
                                transitions={'next':'Park',
                                            'end':'end',
                                            'return':'ON_RAMP'
                                            },
                                remapping={'Park_in_process':'process',
                                           'Park_in_process':'process'})
        smach.StateMachine.add('ON_RAMP', ON_RAMP(),
                                transitions={'end':'end',
                                             'returned':'returned'})

    outcome = sm.execute()
    rospy.spin()
