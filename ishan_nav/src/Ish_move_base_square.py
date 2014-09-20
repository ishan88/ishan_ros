#!/usr/bin/env python




import rospy
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class MoveSquare():
    def __init__(self):
        
        rospy.init_node('move_base_square', anonymous=False)
        
        
        rospy.on_shutdown(self.shutdown)
        
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        
        
        square_size = rospy.get_param("~square_size", 2.0)
        
        
        quaternions = list()
        

        angles = (pi/2, pi, 3*pi/2, 0)
        
        
        for angle in angles:
            
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
            
        
        waypoints = list()
        
        
       # waypoints.append(Pose(Point(0.0, square_size, 0.0),quaternions[0]))
       # waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))
       # waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[2]))
       # waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        
        waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        
        self.init_markers()
        
        
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.marker.points.append(p)
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        
        rospy.loginfo("Waiting for ActionServer")
        
        
        self.move_base.wait_for_server(rospy.Duration(60))
        
        
        rospy.loginfo("Connected to Server")
        
        
        i =0
        
        
        while i<4 and not rospy.is_shutdown():
            
            self.markers_pub.publish(self.marker)

            
            goal = MoveBaseGoal()
            
            
            goal.target_pose.pose = waypoints[i]
            
            
            goal.target_pose.header.frame_id = 'map'
            
            
            goal.target_pose.header.stamp = rospy.Time.now()
            
            
            self.move(goal)
            
            i+=1
             
            
    def move(self, goal): 
        
        self.move_base.send_goal(goal)
        
        rospy.loginfo("Robot Started")
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Goal Cancelled")
        
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal Succeeded")
         
     
    def init_markers(self):
        
        marker_scale = 0.2
        marker_lifetime = 0
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a':1.0}
        
        self.markers_pub = rospy.Publisher('waypoint_markers', Marker) 
        
        self.marker = Marker()
        
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.ns = marker_ns
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.id = marker_id
        self.marker.color.r = marker_color['r']
        self.marker.color.g = marker_color['g']
        self.marker.color.b = marker_color['b']
        self.marker.color.a = marker_color['a']
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        
        self.marker.header.frame_id = 'odom'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.points = list()
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)           
            
            
if __name__ == '__main__':
    try:
        MoveSquare()
    except:
        rospy.loginfo("Node Terminated")
    
        
    

    