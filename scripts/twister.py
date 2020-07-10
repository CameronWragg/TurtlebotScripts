#!/usr/bin/env python
# coding: utf-8

# In[1]:


import rospy
from geometry_msgs.msg import Twist

class CommandVelocity():
    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size = 10)
    
    def send_velocities(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            rospy.loginfo("Sending commands")
            twist_msg = Twist()
            
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = 1.0
            
            self.pub.publish(twist_msg)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities()
    rospy.spin()


# In[ ]:





# In[ ]:





# In[ ]:




