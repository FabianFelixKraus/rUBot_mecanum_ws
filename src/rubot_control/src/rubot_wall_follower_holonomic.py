#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True

    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)


    #back_min1 = int(350 * scanRangesLengthCorrectionFactor)
    #back_max1 = int(360 * scanRangesLengthCorrectionFactor)
    
    #back_min2 = int(0 * scanRangesLengthCorrectionFactor)
    #back_max2 = int(10 * scanRangesLengthCorrectionFactor)
    
    right_min = int(80 * scanRangesLengthCorrectionFactor)
    right_max = int(100 * scanRangesLengthCorrectionFactor)
    
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)

    left_min = int(260 * scanRangesLengthCorrectionFactor)
    left_max = int(280 * scanRangesLengthCorrectionFactor)

    # Concatenate the two back regions and find the minimum range value
    #back_ranges = msg.ranges[back_min1:back_max1] + msg.ranges[back_min2:back_max2]
    #back_range_min = min(back_ranges)
    #back_range_min_capped = min(back_range_min, 3)  # Cap at 3 if needed

    regions = {
        #'back': back_range_min_capped,
        'right': min(min(msg.ranges[right_min:right_max]), 3),
        'front': min(min(msg.ranges[front_min:front_max]), 3),
        'left': min(min(msg.ranges[left_min:left_max]), 3),

        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
    }


    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    linear_y = 0
    rotation_z = 0
    state_description = ''

    if regions['front'] > d and regions['left'] > 2*d and regions['fright'] > 2*d:
        state_description = 'case 1 - nothing'
        linear_x = vx
    else:
        if regions['left'] < d:
            state_description = 'case 3 - left'
            rotation_z = wz
        elif regions['front'] < d:
            state_description = 'case 2 - front'
            linear_y = vx
            if regions['front'] < 0.75*d:
                state_description += ' && too close to wall'
                linear_x = -0.5*vx
        elif regions['fright'] < d:
            state_description = 'case 4 - fright'
            rotation_z = -wz
        else:
            state_description = 'case 5 - Far'
            linear_x = vx

    rospy.loginfo("regions['front'] %5.2f", regions['front'])
    rospy.loginfo("regions['fright'] %5.2f", regions['fright'])
    rospy.loginfo("regions['right'] %5.2f", regions['right'])
    rospy.loginfo("regions['left'] %5.2f", regions['left'])
    rospy.logwarn(state_description)
    rospy.loginfo("x-speed = %5.2f && y-speed = %5.2f", linear_x, linear_y)
    rospy.loginfo("z-rotation-speed = %5.1f", rotation_z)
    rospy.loginfo("----------------------------------")
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = rotation_z
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global vf
    global wz

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    vf= rospy.get_param("~speed_factor")
    wz= rospy.get_param("~rotation_speed")
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()


if __name__ == '__main__':
    main()
