#!/usr/bin/env python

#Script Imports:
import numpy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

#Boolean Array for Colours Detected:
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

#Boolean For Twist Delay:
global waitTime
waitTime = False

#Boolean For Colour Following:
global colourFollow
colourFollow = False

#Boolean For Depth Violation:
global depthL
depthL = False
global depthR
depthR = False

#Value For Centre Pixel Colour:
global centCol
centCol = numpy.zeros((2, 3), dtype=int)
global currCol
currCol = numpy.zeros((1, 3), dtype=int)

#global rate
#rate = 3
#global ctrlRate
#ctrlRate = rospy.Rate(rate)

#Script Class:
class colorDetection:
        #Initialisation
    def __init__(self):
        #-----        
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.twist = Twist()
        #-----



        #Image Callback Function:
    def image_callback(self, msg):
        #Calls Global Variable:        
        global colourStates
        global colourFollow
        global centCol
        #global ctrlRate
        global depthL
        global depthR
        
        #Converts Image Message From Subscribed Topic:
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        
        h, w, d = image.shape
        h = numpy.int(h / 2)
        w = numpy.int(w / 2)
        currCol[0] = numpy.array(hsv[h, w])
        
        #Yellow Detection and Follow        
        if colourStates[0, 0] == True and colourStates[0, 2] == False or colourFollow == False and colourStates[0, 2] == False:   
            lower_yellow = numpy.array([30, 255, 100])
            upper_yellow = numpy.array([30, 255, 210])
            centCol[0] = lower_yellow
            centCol[1] = upper_yellow
            print currCol            
            print centCol
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            h, w, d = image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)               
            if M['m00'] > 0:
                #Extract Centroid Data                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100  
                #print self.twist.angular.z
                if depthL == True and depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.5
                elif depthR == True and depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.5
                self.cmd_vel_pub.publish(self.twist)
                #ctrlRate.sleep()
                if colourStates[0, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'                    
                    colourStates[0, 0] = True
                    colourFollow = True
                    print colourStates
                    
        #Red Detection and Follow
        if colourStates[1, 0] == True and colourStates[1, 2] == False or colourFollow == False and colourStates[1, 2] == False:
            lower_red = numpy.array([0, 255, 100])
            upper_red = numpy.array([0, 255, 210])
            centCol[0] = lower_red
            centCol[1] = upper_red
            print currCol        
            print centCol
            mask = cv2.inRange(hsv, lower_red, upper_red)
            h, w, d = image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)               
            if M['m00'] > 0:
                #Extract Centroid Data                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100
                if depthL == True and depthR == False:
                    self.twist.angular.z = self.twist.angular.z - 0.5
                elif depthR == True and depthL == False:
                    self.twist.angular.z = self.twist.angular.z + 0.5
                #print self.twist.angular.z
                self.cmd_vel_pub.publish(self.twist)
                #ctrlRate.sleep()
                if colourStates[1, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'                    
                    colourStates[1, 0] = True
                    colourFollow = True
                    print colourStates
                    
        #Green Detection and Follow
        if colourStates[2, 0] == True and colourStates[2, 2] == False or colourFollow == False and colourStates[2, 2] == False:
            lower_green = numpy.array([60, 255, 100])
            upper_green = numpy.array([60, 255, 210])
            centCol[0] = lower_green
            centCol[1] = upper_green
            print currCol        
            print centCol
            mask = cv2.inRange(hsv, lower_green, upper_green)
            h, w, d = image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)               
            if M['m00'] > 0:
                #Extract Centroid Data                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100         
                #print self.twist.angular.z
                self.cmd_vel_pub.publish(self.twist)
                #ctrlRate.sleep()
                if colourStates[2, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'                    
                    colourStates[2, 0] = True
                    colourFollow = True
                    print colourStates
                    
        #Blue Detection and Follow
        if colourStates[3, 0] == True and colourStates[3, 2] == False or colourFollow == False and colourStates[3, 2] == False:
            lower_blue = numpy.array([120, 255, 100])
            upper_blue = numpy.array([120, 255, 210])
            centCol[0] = lower_blue
            centCol[1] = upper_blue
            print currCol        
            print centCol
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            h, w, d = image.shape
            search_top = h/2
            search_bot = search_top + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)               
            if M['m00'] > 0:
                #Extract Centroid Data                
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(hsv, (cx, cy), 5, (0, 0, 255), -1)
                err = cx - w/2
                self.twist.linear.x = 0.5
                self.twist.angular.z = -float(err) / 100         
                #print self.twist.angular.z
                if depthL == True and depthR == False:
                    self.twist.linear.x = 0.1                    
                    self.twist.angular.z = self.twist.angular.z - 1.5
                elif depthR == True and depthL == False:
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = self.twist.angular.z + 1.5
                self.cmd_vel_pub.publish(self.twist)
                #ctrlRate.sleep()
                if colourStates[3, 0] == False:
                    print '------------'
                    print 'Colour Found'
                    print '------------'                    
                    colourStates[3, 0] = True
                    colourFollow = True
                    print colourStates
                
        cv2.imshow("window", hsv)
        cv2.waitKey(3)
        
        #Depth Callback Function:
    def depth_callback(self, msgDepth):        
        #Calls Global Variable:
        global colourStates
        global colourFollow
        global waitTime
        global depthL
        global depthR
        #global ctrlRate
        
        #Converts Image Message from Subscribed Topic:
        depth = self.bridge.imgmsg_to_cv2(msgDepth, "32FC1")
        h, w = depth.shape
        x = numpy.int(h / 2)
        #Center of Width
        y = numpy.int(w / 2)
        #Left of Center
        v = numpy.int(w / 4)
        #Right of Center
        u = numpy.int(v * 3)
        
        #Depth Detection:
        self.dDiff = numpy.array([5.0, 5.0])
        for z in numpy.arange(0, v, 1):
            if depth[x, z] < 0.8:
                if depth[x, z] < self.dDiff[0]:
                    self.dDiff[0] = depth[x, z]
                
        for z in numpy.arange(u, w, 1):
            if depth[x, z] < 0.8:
                if depth[x, z] < self.dDiff[1]:
                    self.dDiff[1] = depth[x, z]
        
        if self.dDiff[0] < self.dDiff[1]:
            depthL = True            
            depthR = False
        elif self.dDiff[0] > self.dDiff[1]:
            depthL = False
            depthR = True
        elif self.dDiff[0] == self.dDiff[1] and self.dDiff[0] < 0.8:
            depthL = True
            depthR = True
        else:
            depthL = False
            depthR = False
            
        #if depthL == True and depth[x, y] < 0.8:
        #    depthR = True
        #elif depthR == True and depth[x, y] < 0.8:
        #    depthL = True
        
        #Auto Patrol - Depth Wall Following
        if colourFollow == False:
            if waitTime == False: 
                rospy.sleep(2)
                print 'No Colours Found - Patrolling'
                waitTime = True
                self.twist.angular.z = 0.0
                self.twist.linear.x =  0.0
            if depthL == True and depthR == True:
                self.twist.angular.z = 0.0
                self.twist.linear.x = -2.0
            elif depthL == True and depthR == False:
                if self.dDiff[0] < depth[x, y]:
                    self.twist.angular.z = 1.5
                    self.twist.linear.x = -0.1
                else:
                    self.twist.angular.z = 1.5
                    self.twist.linear.x = -0.1
            elif depthR == True and depthL == False:
                if self.dDiff[1] < depth[x, y]:
                    self.twist.angular.z = -1.5
                    self.twist.linear.x = -0.1
                else:
                    self.twist.angular.z = -1.5
                    self.twist.linear.x = -0.1
            elif depthL == False and depthR == False:
                self.dLStrength = 0
                self.dRStrength = 0                
                for z in numpy.arange(0, w, 1):
                    if depth[x, z] > 2.5:
                        if z < y:
                            self.dLStrength = self.dLStrength + 1
                        elif z > y:
                            self.dRStrength = self.dRStrength + 1
                        else:
                            self.dLStrength = self.dLStrength + 1
                            self.dRStrength = self.dRStrength + 1
                if self.dLStrength > self.dRStrength and depth[x, y] > 2.5:
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.5
                elif self.dLStrength < self.dRStrength and depth[x, y] > 2.5:
                    self.twist.angular.z = 0.01
                    self.twist.linear.x = 0.5
                elif self.dLStrength == self.dRStrength and depth[x, y] > 2.5:
                    self.twist.angular.z = 0.0
                    self.twist.linear.x = 0.5
                elif self.dLStrength > self.dRStrength and depth[x, y] < 2.5:
                    self.twist.angular.z = -0.5
                    self.twist.linear.x = -0.5
                elif self.dLStrength < self.dRStrength and depth[x, y] < 2.5:
                    self.twist.angular.z = 0.5
                    self.twist.linear.x = -0.5
                elif self.dLStrength == self.dRStrength and depth[x, y] < 2.5:
                    self.twist.angular.z = 0
                    self.twist.linear.x = -0.5
            #depthL = False
            #depthR = False
            self.cmd_vel_pub.publish(self.twist)
            
        
        #Yellow Depth Detection        
        if colourStates[0, 0] == True and colourStates[0, 1] == False:       
            if depth[x, y] <= 1.0:
                if centCol[0, 0] == 30 and centCol[0, 1] == 255:
                    for z in numpy.arange(centCol[0, 2], centCol[1, 2], 1):
                        if currCol[0, 2] == z:
                            colourStates[0, 1] = True
                            depthL = True                            
                            depthR = True
                            print '------------'
                            print 'Close Proxim'
                            print '------------'
                            print colourStates
                            break
        elif colourStates[0, 0] == True and colourStates[0, 1] == True and colourStates[0, 2] == False:
            colourStates[0, 2] = True
            print 'Yellow Post Detected and in Close Proximity.'
            print colourStates
            colourFollow = False
            waitTime = False
            
        #Red Depth Detection
        if colourStates[1, 0] == True and colourStates[1, 1] == False:       
            if depth[x, y] <= 1.0:
                if centCol[0, 0] == 0 and centCol[0, 1] == 255:
                    for z in numpy.arange(centCol[0, 2], centCol[1, 2], 1):
                        if currCol[0, 2] == z:                
                            colourStates[1, 1] = True
                            depthL = True                            
                            depthR = True
                            print '------------'
                            print 'Close Proxim'
                            print '------------'
                            print colourStates
        elif colourStates[1, 0] == True and colourStates[1, 1] == True and colourStates[1, 2] == False:
            colourStates[1, 2] = True
            print 'Red Post Detected and in Close Proximity.'
            print colourStates
            colourFollow = False
            waitTime = False
            self.twist.angular.z = 1.5
            self.twist.linear.x = -0.1
            self.cmd_vel_pub.publish(self.twist)
            
        #Green Depth Detection
        if colourStates[2, 0] == True and colourStates[2, 1] == False:       
            if depth[x, y] <= 1.0:
                if centCol[0, 0] == 60 and centCol[0, 1] == 255:
                    for z in numpy.arange(centCol[0, 2], centCol[1, 2], 1):
                        if currCol[0, 2] == z:                
                            colourStates[2, 1] = True
                            depthR = True
                            print '------------'
                            print 'Close Proxim'
                            print '------------'
                            print colourStates
        elif colourStates[2, 0] == True and colourStates[2, 1] == True and colourStates[2, 2] == False:
            colourStates[2, 2] = True
            print 'Green Post Detected and in Close Proximity.'
            print colourStates
            colourFollow = False
            waitTime = False
            
        #Blue Depth Detection
        if colourStates[3, 0] == True and colourStates[3, 1] == False:       
            if depth[x, y] <= 1.0:
                if centCol[0, 0] == 120 and centCol[0, 1] == 255:
                    for z in numpy.arange(centCol[0, 2], centCol[1, 2], 1):
                        if currCol[0, 2] == z:                
                            colourStates[3, 1] = True
                            depthL                            
                            depthR = True
                            print '------------'
                            print 'Close Proxim'
                            print '------------'
                            print colourStates
        elif colourStates[3, 0] == True and colourStates[3, 1] == True and colourStates[3, 2] == False:
            colourStates[3, 2] = True
            print 'Blue Post Detected and in Close Proximity.'
            print colourStates
            colourFollow = False
            waitTime = False
        
        

cv2.startWindowThread()
rospy.init_node('colDet')
colDet = colorDetection()
rospy.spin()

cv2.destroyAllWindows()