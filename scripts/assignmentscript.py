#!/usr/bin/env python

#--------------------
#Cameron Stuart Wragg
#----- 15595673 -----
#--------------------
#Script Imports:
import numpy
import cv2
import cv_bridge
import rospy
import actionlib
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Boolean Array for Colour Detection and Proximity States:
global colourStates
colourStates = numpy.full((4, 3), False, dtype=bool)
#-----COLOUR STATES GUIDE:
#--
#---[0, 0] = Yellow Found
#---[0, 1] = Yellow In Proximity
#---[0, 2] = Yellow Message Displayed
#--
#---[1, 0] = Red Found
#---[1, 1] = Red In Proximity
#---[1, 2] = Red Message Displayed
#--
#---[2, 0] = Green Found
#---[2, 1] = Green In Proximity
#---[2, 2] = Green Message Displayed
#--
#---[3, 0] = Blue Found
#---[3, 1] = Blue In Proximity
#---[3, 2] = Blue Message Displayed
#--

#Script Class:
class colorDetection:
        #Initialisation:
    def __init__(self):
        #CvBridge For Converting OpenCV Images to ROS Image Messages:
        self.bridge = cv_bridge.CvBridge()
        #Subscriber to RAW RGB Image Topic:
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        #Publisher to Mobile Base Velocity Command Topic (Moving Turtlebot):
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        #Subscriber to RAW Depth Image Topic:
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        #Calling Twist Function:
        self.twist = Twist()
        #Defining an Array of Waypoints:
        self.waypoints = [
                ['0', (-0.636, 1.955, 0.00), (0.0, 0.0, 0.106, 0.994)],
                ['1', (2.918, 1.940, 0.00), (0.0, 0.0, -0.619, 0.785)],
                ['2', (3.237, -1.655, 0.00), (0.0, 0.0, 0.995, -0.099)],
                ['3', (1.449, -2.416, 0.00), (0.0, 0.0, -0.772, 0.636)],
                ['4', (-0.935, -4.350, 0.00), (0.0, 0.0, 0.994, -0.112)],
                ['5', (1.453, -4.467, 0.00), (0.0, 0.0, -0.135, 0.991)],
                ['6', (-1.248, -1.679, 0.00), (0.0, 0.0, 0.992, 0.124)],
                ['7', (-4.210, -0.936, 0.00), (0.0, 0.0, 0.591, 0.807)],
                ['8', (-4.097, 1.449, 0.00), (0.0, 0.0, 0.573, 0.819)],
                ['9', (-4.258, 4.021, 0.00), (0.0, 0.0, 0.707, 0.707)],
                ['10', (-1.027, 5.291, 0.00), (0.0, 0.0, -0.505, 0.864)],
                ['11', (1.463, 4.499, 0.00), (0.0, 0.0, -0.170, 0.985)]
        ]
        #Defining Waypoint for Last Waypoint Given:
        self.lastwaypoint = [
                ['11', (1.463, 4.499, 0.00), (0.0, 0.0, -0.170, 0.985)]
        ]
        #Defining Waypoint for Last Waypoint Reached:
        self.goalmet = [
                ['0', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)]
        ]
        #Array for Central Pixel Colour Boundaries:
        self.centCol = numpy.zeros((2, 3), dtype=int)
        #Array for Current Central Pixel Colour:
        self.currCol = numpy.zeros((1, 3), dtype=int)
        #MoveBase Action Library Client:
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #Wait for Server Function:
        self.client.wait_for_server()
        #Define RGB Image:
        self.image = Image
        #Define Depth Image:
        self.depth = Image
        #Boolean for Left Side Depth Detection:
        self.depthL = False
        #Boolean for Right Side Depth Detection:
        self.depthR = False
        #Boolean for Depth Proximity to Coloured Post:
        self.depthState = False
        #Boolean for Current Colour Following State:
        self.colourFollow = 'False'
        #Call Patrol Function at Initialisation:
        self.patrol_start(self.colourFollow)
        #Sleep so Timer Isn't Initialised Too Early:
        rospy.sleep(2)
        #Timer Callback for Patrol Function:
        self.colfol_timer = rospy.Timer(rospy.Duration(1), self.patrol_start)



        #Patrol MoveBase Function:
    def patrol_wp(self, pose):
        #Passes Waypoint Co Ordinates Correctly to MoveBase Client:
        patrol_wp = MoveBaseGoal()
        patrol_wp.target_pose.header.frame_id = 'map'
        patrol_wp.target_pose.pose.position.x = pose[1][0]
        patrol_wp.target_pose.pose.position.y = pose[1][1]
        patrol_wp.target_pose.pose.position.z = pose[1][2]
        patrol_wp.target_pose.pose.orientation.x = pose[2][0]
        patrol_wp.target_pose.pose.orientation.y = pose[2][1]
        patrol_wp.target_pose.pose.orientation.z = pose[2][2]
        patrol_wp.target_pose.pose.orientation.w = pose[2][3]
        return patrol_wp



        #Patrol Timed Callback Function:
    def patrol_start(self, time):
        #Calls Global Variable:
        global colourStates

        x = int(self.lastwaypoint[0][0])
        y = int(self.goalmet[0][0])
        if x != y:
            if self.colourFollow == 'False':
                #FINAL GOAL STATE CHECK:
                if colourStates[0, 2] == True and colourStates[1, 2] == True and colourStates[2, 2] == True and colourStates[3, 2] == True:
                    print '\n\n\n'
                    print colourStates
                    print '================================================'
                    print 'All Coloured Posts Found within Close Proximity.'
                    print '================================================'
                    cv2.destroyAllWindows()
                    sys.exit(0)
                #=======================
                if x == 11:
                    self.lastwaypoint = self.waypoints[0]
                    goal = self.patrol_wp(self.lastwaypoint)
                    self.goalmet[0][0] = self.lastwaypoint[0][0]
                    x = int(self.lastwaypoint[0][0])
                    self.lastwaypoint = self.waypoints[(x + 1)]
                    self.client.send_goal(goal)
                    self.client.wait_for_result()
                    self.colour_dandf(self.image)
                    if self.colourFollow == 'False':
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 1.5
                        self.cmd_vel_pub.publish(self.twist)
                        self.colour_dandf(self.image)
                elif x != 11:
                    goal = self.patrol_wp(self.lastwaypoint)
                    self.goalmet[0][0] = self.lastwaypoint[0][0]
                    x = int(self.lastwaypoint[0][0])
                    self.lastwaypoint = self.waypoints[(x + 1)]
                    self.client.send_goal(goal)
                    self.client.wait_for_result()
                    self.colour_dandf(self.image)
                    if self.colourFollow == 'False':
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 1.5
                        self.cmd_vel_pub.publish(self.twist)
                        self.colour_dandf(self.image)
            elif self.colourFollow == 'True':
                self.colour_dandf(self.image)
                self.depth_dandc(self.depth)



        #Image Callback Function:
    def image_callback(self, msg):
        #Converts Image Message From Subscribed Topic:
        cv2.namedWindow("window", 1)
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        h, w, d = hsv.shape
        h = numpy.int(h / 2)
        w = numpy.int(w / 2)
        self.currCol[0] = numpy.array(hsv[h, w])
        cv2.imshow("window", hsv)
        cv2.waitKey(3)



        #Colour Detection and Follow Function:
    def colour_dandf(self, image):
        #Calls Global Variable:
        global colourStates

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, d = self.image.shape
        h = numpy.int(h / 2)
        w = numpy.int(w / 2)

        #Yellow Detection and Follow
        if colourStates[0, 0] == True and colourStates[0, 2] == False or self.colourFollow == 'False' and colourStates[0, 2] == False:
            lower_yellow = numpy.array([30, 235, 100])
            upper_yellow = numpy.array([30, 255, 210])
            self.centCol[0] = lower_yellow
            self.centCol[1] = upper_yellow
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            h, w, d = self.image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                self.client.cancel_all_goals
                #Extract Centroid Data
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                if self.depthL == True and self.depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.2
                elif self.depthR == True and self.depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.2
                self.cmd_vel_pub.publish(self.twist)
                if colourStates[0, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'
                    colourStates[0, 0] = True
                    self.colourFollow = 'True'
                    print colourStates
                if colourStates[0, 2] == False:
                    self.depthState = True
                    rospy.sleep(0.2)
                    self.depth_dandc(self.depth)

        #Red Detection and Follow
        if colourStates[1, 0] == True and colourStates[1, 2] == False or self.colourFollow == 'False' and colourStates[1, 2] == False:
            lower_red = numpy.array([0, 235, 100])
            upper_red = numpy.array([0, 255, 210])
            self.centCol[0] = lower_red
            self.centCol[1] = upper_red
            mask = cv2.inRange(hsv, lower_red, upper_red)
            h, w, d = self.image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                self.client.cancel_all_goals
                #Extract Centroid Data
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                if self.depthL == True and self.depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.2
                elif self.depthR == True and self.depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.2
                self.cmd_vel_pub.publish(self.twist)
                if colourStates[1, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'
                    colourStates[1, 0] = True
                    self.colourFollow = 'True'
                    print colourStates
                if colourStates[1, 2] == False:
                    self.depthState = True
                    rospy.sleep(0.2)
                    self.depth_dandc(self.depth)

        #Green Detection and Follow
        if colourStates[2, 0] == True and colourStates[2, 2] == False or self.colourFollow == 'False' and colourStates[2, 2] == False:
            lower_green = numpy.array([60, 235, 100])
            upper_green = numpy.array([60, 255, 210])
            self.centCol[0] = lower_green
            self.centCol[1] = upper_green
            mask = cv2.inRange(hsv, lower_green, upper_green)
            h, w, d = self.image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                self.client.cancel_all_goals
                #Extract Centroid Data
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                if self.depthL == True and self.depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.2
                elif self.depthR == True and self.depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.2
                self.cmd_vel_pub.publish(self.twist)
                if colourStates[2, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'
                    colourStates[2, 0] = True
                    self.colourFollow = 'True'
                    print colourStates
                if colourStates[2, 2] == False:
                    self.depthState = True
                    rospy.sleep(0.2)
                    self.depth_dandc(self.depth)

        #Blue Detection and Follow
        if colourStates[3, 0] == True and colourStates[3, 2] == False or self.colourFollow == 'False' and colourStates[3, 2] == False:
            lower_blue = numpy.array([120, 235, 100])
            upper_blue = numpy.array([120, 255, 210])
            self.centCol[0] = lower_blue
            self.centCol[1] = upper_blue
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            h, w, d = self.image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            if M['m00'] > 0:
                self.client.cancel_all_goals
                #Extract Centroid Data
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                if self.depthL == True and self.depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.2
                elif self.depthR == True and self.depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.2
                self.cmd_vel_pub.publish(self.twist)
                if colourStates[3, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'
                    colourStates[3, 0] = True
                    self.colourFollow = 'True'
                    print colourStates
                if colourStates[3, 2] == False:
                    self.depthState = True
                    rospy.sleep(0.2)
                    self.depth_dandc(self.depth)



        #Depth Callback Function:
    def depth_callback(self, msgDepth):
        #Calls Global Variable:
        global colourStates

        #Converts Image Message from Subscribed Topic:
        self.depth = self.bridge.imgmsg_to_cv2(msgDepth, "32FC1")
        #Pulls Height and Width from Image Shape:
        h, w = self.depth.shape
        #Half of Height:
        x = numpy.int(h / 2)
        #Left of Center:
        v = numpy.int(w / 4)
        #Right of Center:
        u = numpy.int(v * 3)

        #Depth Detection for Left and Right Quadrant:
        self.dDiff = numpy.array([5.0, 5.0])
        for z in numpy.arange(0, v, 1):
            if self.depth[x, z] < 0.8:
                if self.depth[x, z] < self.dDiff[0]:
                    self.dDiff[0] = self.depth[x, z]

        for z in numpy.arange(u, w, 1):
            if self.depth[x, z] < 0.8:
                if self.depth[x, z] < self.dDiff[1]:
                    self.dDiff[1] = self.depth[x, z]

        if self.dDiff[0] < self.dDiff[1]:
            self.depthL = True
            self.depthR = False
        elif self.dDiff[0] > self.dDiff[1]:
            self.depthL = False
            self.depthR = True
        elif self.dDiff[0] == self.dDiff[1] and self.dDiff[0] < 0.8:
            self.depthL = True
            self.depthR = True
        else:
            self.depthL = False
            self.depthR = False



        #Depth Detection and Collision Function:
    def depth_dandc(self, depth):
        #Calls Global Variable:
        global colourStates

        #Height and Width of Depth Image Shape:
        h, w = depth.shape
        #Half of Height:
        x = numpy.int(h / 2)
        #Center of Width:
        y = numpy.int(w / 2)

        #Yellow Depth Detection
        if colourStates[0, 0] == True and colourStates[0, 1] == False:
            if depth[x, y] <= 1.0:
                if self.centCol[0, 0] == 30:
                    colourStates[0, 1] = True
                    print '------------'
                    print 'Close Proxim'
                    print '------------'
                    print colourStates
        if colourStates[0, 0] == True and colourStates[0, 1] == True and colourStates[0, 2] == False:
            print '\nYellow Post Detected and in Close Proximity.\n'
            colourStates[0, 2] = True
            print colourStates
            self.colourFollow = 'False'
            self.depthState = False

        #Red Depth Detection
        if colourStates[1, 0] == True and colourStates[1, 1] == False:
            if depth[x, y] <= 1.0:
                if self.centCol[0, 0] == 0:
                    colourStates[1, 1] = True
                    print '------------'
                    print 'Close Proxim'
                    print '------------'
                    print colourStates
        if colourStates[1, 0] == True and colourStates[1, 1] == True and colourStates[1, 2] == False:
            print '\nRed Post Detected and in Close Proximity.\n'
            colourStates[1, 2] = True
            print colourStates
            self.colourFollow = 'False'
            self.depthState = False

        #Green Depth Detection
        if colourStates[2, 0] == True and colourStates[2, 1] == False:
            if depth[x, y] <= 1.0:
                if self.centCol[0, 0] == 60:
                    colourStates[2, 1] = True
                    print '------------'
                    print 'Close Proxim'
                    print '------------'
                    print colourStates
        if colourStates[2, 0] == True and colourStates[2, 1] == True and colourStates[2, 2] == False:
            print '\nGreen Post Detected and in Close Proximity.\n'
            colourStates[2, 2] = True
            print colourStates
            self.colourFollow = 'False'
            self.depthState = False

        #Blue Depth Detection
        if colourStates[3, 0] == True and colourStates[3, 1] == False:
            if depth[x, y] <= 1.0:
                if self.centCol[0, 0] == 120:
                    colourStates[3, 1] = True
                    print '------------'
                    print 'Close Proxim'
                    print '------------'
                    print colourStates
        if colourStates[3, 0] == True and colourStates[3, 1] == True and colourStates[3, 2] == False:
            print '\nBlue Post Detected and in Close Proximity.\n'
            colourStates[3, 2] = True
            print colourStates
            self.colourFollow = 'False'
            self.depthState = False

        #If Statement at end of Depth Detection and Collision Function:
        if self.depthState == True:
            self.colour_dandf(self.image)



#MAIN:
if __name__ == '__main__':
    cv2.startWindowThread()
    rospy.init_node('colDet')
    colDet = colorDetection()
    rospy.spin()
    cv2.destroyAllWindows()


#+++++ REFERENCES +++++
#Quigley, M., Gerkey, B. and Smart, W.D. (2015) Navigating About the World.
#In: Programming Robots with ROS: A Practical Introduction to the Robot Operating System.
#California, USA: O'Reilly Media, Inc, 151-163.

#Quigley, M., Gerkey, B. and Smart, W.D. (2015) Follow-bot.
#In: Programming Robots with ROS: A Practical Introduction to the Robot Operating System.
#California, USA: O'Reilly Media, Inc, 193-208.
#++++++++++++++++++++++



#========== EXPERIMENTAL CODE || IGNORE ==========

#Auto Patrol - Depth Wall Following
#if colourFollow == False:
#    if waitTime == False:
#        rospy.sleep(2)
#        print 'No Colours Found - Patrolling'
#        waitTime = True
#        self.twist.angular.z = 0.0
#        self.twist.linear.x =  0.0
#    if depthL == True and depthR == True:
#        self.twist.angular.z = 0.0
#        self.twist.linear.x = -2.0
#    elif depthL == True and depthR == False:
#        if self.dDiff[0] < depth[x, y]:
#            self.twist.angular.z = 1.5
#            self.twist.linear.x = -0.1
#        else:
#            self.twist.angular.z = 1.5
#            self.twist.linear.x = -0.1
#    elif depthR == True and depthL == False:
#        if self.dDiff[1] < depth[x, y]:
#            self.twist.angular.z = -1.5
#            self.twist.linear.x = -0.1
#        else:
#            self.twist.angular.z = -1.5
#            self.twist.linear.x = -0.1
#    elif depthL == False and depthR == False:
#        self.dLStrength = 0
#        self.dRStrength = 0
#        for z in numpy.arange(0, w, 1):
#            if depth[x, z] > 2.5:
#                if z < y:
#                    self.dLStrength = self.dLStrength + 1
#                elif z > y:
#                    self.dRStrength = self.dRStrength + 1
#                else:
#                    self.dLStrength = self.dLStrength + 1
#                    self.dRStrength = self.dRStrength + 1
#        if self.dLStrength > self.dRStrength and depth[x, y] > 2.5:
#            self.twist.angular.z = 0.0
#            self.twist.linear.x = 0.5
#        elif self.dLStrength < self.dRStrength and depth[x, y] > 2.5:
#            self.twist.angular.z = 0.01
#            self.twist.linear.x = 0.5
#        elif self.dLStrength == self.dRStrength and depth[x, y] > 2.5:
#            self.twist.angular.z = 0.0
#            self.twist.linear.x = 0.5
#        elif self.dLStrength > self.dRStrength and depth[x, y] < 2.5:
#            self.twist.angular.z = -0.5
#            self.twist.linear.x = -0.5
#        elif self.dLStrength < self.dRStrength and depth[x, y] < 2.5:
#            self.twist.angular.z = 0.5
#            self.twist.linear.x = -0.5
#        elif self.dLStrength == self.dRStrength and depth[x, y] < 2.5:
#            self.twist.angular.z = 0
#            self.twist.linear.x = -0.5
#    self.cmd_vel_pub.publish(self.twist)
