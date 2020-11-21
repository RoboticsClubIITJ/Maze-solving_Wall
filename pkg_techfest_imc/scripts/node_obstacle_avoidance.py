#! /usr/bin/env python

import rospy
import statistics
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def clbk_laser(msg):
    regions = {
        'right':  min(statistics.mean(msg.ranges[0:144]), 10),
        'fright': min(statistics.mean(msg.ranges[145:288]), 10),
        'front':  min(statistics.mean(msg.ranges[289:432]), 10),
        'fleft':  min(statistics.mean(msg.ranges[433:576]), 10),
        'left':   min(statistics.mean(msg.ranges[577:719]), 10),
    }
    
    take_action(regions)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    d = 0.22
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        linear_x = 0
        angular_z = 0.6
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.6
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.6
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        linear_x = 0.3
        angular_z = 0.0
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.6
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.6
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.6
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub
    
    rospy.init_node('obstacle')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
