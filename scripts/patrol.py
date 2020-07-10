#!/usr/bin/env python

#Script Imports:
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

wayPoints = [
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

def patrol_wp(pose):
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
        
if __name__ == '__main__':
    rospy.init_node('patrol')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    while True:
        for pose in wayPoints:
            goal = patrol_wp(pose)
            client.send_goal(goal)
            client.wait_for_result()