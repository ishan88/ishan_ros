#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        
        rospy.loginfo("Robot Started")
        rospy.init_node('outandback', anonymous=False)
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        
        rospy.on_shutdown(self.shutdown)
        
        rate = 50
        
        r = rospy.Rate(rate)
        
        linear_speed = 0.2
        
        goal_distance = 5.0
        
        linear_duration = goal_distance/linear_speed
        
        angular_speed = 1.0
        
        goal_angle = pi
        
        angular_duration = goal_angle/angular_speed
        
        for i in range(2):
            
            move_cmd = Twist()
            
            move_cmd.linear.x = linear_speed
            
            ticks = int(linear_duration * rate)
            
            for t in range(ticks):
                
                self.cmd_vel.publish(move_cmd)
                
                r.sleep()
           
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            
            move_cmd = Twist()
            move_cmd.angular.z = angular_speed
            
            ticks = int(goal_angle * rate)
            
            for t in range(ticks):
                
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
        
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    
if __name__ == '__main__':
    OutAndBack()
       # except:
        #    rospy.loginfo("Error Running the Node")
        
            