#! /usr/bin/env python
import time
import rospy
import statistics
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
msg = Twist()

def clbk_laser(msg):
    regions = {
        'right':  min(statistics.mean(msg.ranges[0:50]), 10),
        'fright': min(statistics.mean(msg.ranges[51:100]), 10),
        'front':  min(statistics.mean(msg.ranges[147:219]), 10),
        'fleft':  min(statistics.mean(msg.ranges[259:308]), 10),
        'left':   min(statistics.mean(msg.ranges[309:359]), 10),
    }



a = 0
def rotate():
    global msg, pub
    
    


def take_action(regions):
    global msg,pub 
    linear_x = 0
    angular_z = 0

    state_description = ''
    d = 0.15
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        linear_x = 0
        angular_z = 0

    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0

    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0
        duration = 2
        msg.angular.z = ((3.14)/2)/(duration)

        pub.publish(msg)
        time.sleep(duration)
        

        msg.angular.z = 0
        pub.publish(msg)

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        linear_x = 0.15
        angular_z = 0.0

    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0

    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = 0

    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0

    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.15
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

    rospy.init_node('try3')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    take_action(regions)
    rospy.spin()


if __name__ == '__main__':
    main()
    
