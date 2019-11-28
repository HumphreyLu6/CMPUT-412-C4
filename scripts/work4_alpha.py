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
            if self.box_tag_id != None: # and self.goal_tag_id != None:
                break
            # twist = Twist()
            # twist.angular.z = 0.2
            # twist_pub.publish(twist)

            # if time.time() - tmp_time > 5:
            #     goal_pose = util.goal_pose('map', PARK_SPOT_WAYPOINTS['6'][0], PARK_SPOT_WAYPOINTS['6'][1])
            #     self.client.send_goal(goal_pose)
            #     self.client.wait_for_result()
            #     util.rotate(87)
            
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
                # (trans,rots) = self.listener.lookupTransform('/map', '/ar_marker_' + str(self.goal_tag_id), rospy.Time(0))
                # point = Point(trans[0], trans[1], 0.010)
                # # quaternion = Quaternion(rots[0],rots[1],rots[2],rots[3])
                # quaternion = Quaternion(0, 0, 0.118, 0.993)
                # self.goal_waypoint = util.goal_pose('map', point, quaternion)
                # print "self.goal_waypoint", self.goal_waypoint
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            finally:
                if self.box_waypoint != None:# and self.goal_waypoint != None:
                    break
                print "Try to get waypoints:", self.box_waypoint, self.goal_waypoint
        
        
        # print "goal stall id:", get_cloest_stall(self.goal_waypoint.target_pose.pose.position)

        # assert self.box_waypoint != None and self.goal_waypoint != None

        # self.box_stall_id = get_cloest_stall(self.box_waypoint.target_pose.pose.position)
        # self.goal_stall_id = get_cloest_stall(self.goal_waypoint.target_pose.pose.position)
        # print self.box_stall_id, self.goal_stall_id
        # assert self.box_stall_id != self.goal_stall_id

        # box_is_left = True
        # if int(self.box_stall_id) > int(self.goal_stall_id):
        #     box_is_left = False

        # square_dist = 0.825
        # if box_is_left:
        #     point = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)-1)][0]
        #     quaternion = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)-1)][1]
        #     goal_pose = util.goal_pose('map', point, quaternion)
        #     self.client.send_goal(goal_pose)
        #     self.client.wait_for_result()
        #     util.signal(1, onColor=Led.BLACK) #debug
        #     util.rotate(-87)
        #     push_dist = (abs( int(self.box_stall_id)- int(self.goal_stall_id)) + 1) * square_dist - 0.4
        #     util.move(push_dist)
        #     util.signal(2, onColor=Led.GREEN)
        #     util.rotate(87)
        # else:
        #     point = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)+1)][0]
        #     quaternion = PARK_SPOT_WAYPOINTS[str(int(self.box_stall_id)+1)][1]
        #     goal_pose = util.goal_pose('map', point, quaternion)
        #     self.client.send_goal(goal_pose)
        #     self.client.wait_for_result()
        #     util.signal(1, onColor=Led.BLACK) #debug
        #     util.rotate(87)
        #     push_dist = abs( int(self.box_stall_id)- int(self.goal_stall_id) + 1) * square_dist
        #     util.move(push_dist)
        #     util.signal(2, onColor=Led.GREEN)
        #     util.rotate(-87)

        self.go_to_box_sides('box_left')

        ar_tag_sub.unregister()
        return 'completed'
  
    def ar_tag_sub_callback(self, msg):
        for marker in msg.markers:
            if marker.id == 6 and self.box_tag_id == None:
                self.box_tag_id = marker.id
                #util.signal(quantity=1, onColor=Led.RED)
            if marker.id == 30 and self.goal_tag_id == None:
                self.goal_tag_id = marker.id
                #util.signal(quantity=1, onColor=Led.GREEN)
    
    def set_init_map_pose(self):
        #referenced from https://www.cnblogs.com/kuangxionghui/p/8335853.html

        init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)
        rospy.loginfo("start set pose...")
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        # p.pose.pose.position = Point(-0.40129, -3.2305, 0.0102) #OFF_RAMP_WAYPOINT[0]
        # p.pose.pose.orientation = Quaternion(0, 0, 0.70878, 0.70543) #OFF_RAMP_WAYPOINT[1]
        p.pose.pose.position = OFF_RAMP_WAYPOINT[0]
        p.pose.pose.orientation = OFF_RAMP_WAYPOINT[1]

        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

        init_pose_pub.publish(p)
        rospy.sleep(3)
    
    def go_to_box_sides(self, position):
        br = tf.TransformBroadcaster()
        listener = tf.TransformListener()

        side_offset_from_middle = 0.5
        middle_offset_from_relative= (0, 0, -0.167)
        relative_rotation = (0, 0, 0, 1)

        # point (x+=>x-, y+=>z-, z+=>y-)
        sides = {"box_front":[(0, 0, side_offset_from_middle ), (0, 0.707, 0, 0.707)],
                 "box_left": [(-side_offset_from_middle, 0, 0), (0, 0, 0, 1)],
                 "box_right":[(side_offset_from_middle, 0,  0), (0, 0, 1, 0)]}
        
        # sides = {"box_front":[(0, 0,  side_offset_from_middle), (0, 0, 0, 1)],
        #          "box_left": [(0, -side_offset_from_middle, 0), (0, 0, 0.707, 0.707)],
        #          "box_right":[(0,  side_offset_from_middle, 0), (0, 0, -0.707, 0.707)]}

        assert position in sides

        tmp_time = time.time()
        while True or (time.time() - tmp_time) > 100:
            br.sendTransform(
                middle_offset_from_relative,
                relative_rotation,
                rospy.Time.now(),
                "box_middle",
                "ar_marker_6",
            )
            #position = "box_left"
            br.sendTransform(
                sides[position][0],
                sides[position][1],
                rospy.Time.now(),
                position,
                "box_middle",
            )

            # position = "box_right"
            # br.sendTransform(
            #     # front
            #     sides[position][0],
            #     sides[position][1],
            #     rospy.Time.now(),
            #     position,
            #     "box_middle",
            # )

            # position = "box_front"
            # br.sendTransform(
            #     # front
            #     sides[position][0],
            #     sides[position][1],
            #     rospy.Time.now(),
            #     position,
            #     "box_middle",
            # )
            rospy.sleep(0.2)
            try:
                (trans,rot) = listener.lookupTransform('/map', position, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("look up exceptions")
                continue
            else:
                # point = Point(trans[0], trans[1], trans[2])
                # quaternion = Quaternion(rot[0],rot[1],rot[2],rot[3])
                # quaternion = Quaternion(0, 0, 0.707, 0.707)
                point = Point(0, 0, 0)
                quaternion = Quaternion(0, 0, 0, 1)
                goal_pose = util.goal_pose(position, point, quaternion)
                self.client.send_goal(goal_pose)
                self.client.wait_for_result()
                util.signal(1)
                break
            

if __name__ == "__main__":
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end', 'returned'])
    sm.userdata.contour = detectshapes.Contour.Triangle

    with sm:
        smach.StateMachine.add('PushBox', PushBox(),
                                transitions={'completed':'end',
                                            'end':'end'
                                            })

    outcome = sm.execute()
    rospy.spin()
