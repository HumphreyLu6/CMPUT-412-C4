import ros, rospy, numpy, cv2, cv_bridge, time, sys
from sensor_msgs.msg import Image, LaserScan
from detectshapes import ContourDetector, threshold_hsv_360

def image_callback(msg):
    global g_hsv
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    #image = cv2.pyrMeanShiftFiltering(image, 15, 20)  #10 20 for red
    g_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #green mask
    lower_green = numpy.array([40, 50, 50]) 
    upper_green = numpy.array([80, 255, 255])
    green_mask = cv2.inRange(g_hsv, lower_green, upper_green)

    #red mask
    red_mask = threshold_hsv_360(150, 100, 20, 255, 255, 320, g_hsv) #20, 320

    #cv2.imshow("g_hsv", g_hsv)

    #cv2.imshow("green_mask", green_mask)    
    #cv2.moveWindow("green_mask", 710, 0)

    #cv2.imshow("red_mask", red_mask)
    #cv2.moveWindow("red_mask", 710, 0) 
    #cv2.waitKey(3)

def laser_callback(msg):
    #g_test = msg.ranges
    print len(msg.ranges)
    print msg.range_min
    print msg.range_max
    array = numpy.array(msg.ranges[280: 381])
    array = array[~numpy.isnan(array)]
    print numpy.mean(array) 

def main(argv):
    global cd, g_hsv

if __name__ == "__main__":
    cd = ContourDetector()
    g_hsv = None

    cd = ContourDetector()
    rospy.init_node("test_node")
    image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    rospy.wait_for_message('camera/rgb/image_raw', Image)
    #rospy.Subscriber("/scan", LaserScan, laser_callback)
    
    s_time = time.time()
    while not rospy.is_shutdown():
        cd.getContours(g_hsv, int(sys.argv[1]))
        print time.time()
        # if (time.time() - s_time) >= 3:
        #     cd.getContours(g_hsv, int(sys.argv[1]))
        #     s_time = time.time()
    rospy.spin()