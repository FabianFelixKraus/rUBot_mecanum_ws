#!/usr/bin/env python3
import rospy
from rubot_nav import move_rubot
from math import sqrt, sin, cos, pi, radians

def square_path(v, td):
    move_rubot(v, 0, 0, td)
    move_rubot(0, v, 0, td)
    move_rubot(-v, 0, 0, td)
    move_rubot(0, -v, 0, td)

def triangular_path(v, td):
    move_rubot(v, 0, 0, td)
    move_rubot(-v, v, 0, td/sqrt(2))
    move_rubot(-v, -v, 0, td/sqrt(2))

def star_path(v, td, star_tips):
    print("star_tips", star_tips)
    angle = 360/star_tips
    for i in range(star_tips):
        rospy.loginfo("Star tip number %i with coord: (%f, %f)", i+1, v*cos(radians(angle*i)), v*sin(radians(angle*i)))
    for i in range(star_tips):
        move_rubot(v*cos(radians(angle*i)), v*sin(radians(angle*i)), 0, td)
        move_rubot(-v*cos(radians(angle*i)), -v*sin(radians(angle*i)), 0, td)

def rhombus_path(v, td):
    move_rubot(0.5*v, v, 0, td)
    move_rubot(v, 0.5*v, 0, td)
    move_rubot(-0.5*v, -v, 0, td)
    move_rubot(-v, -0.5*v, 0, td)

if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        v = rospy.get_param("~v")
        w = rospy.get_param("~w")
        td = rospy.get_param("~td")
        path = rospy.get_param("~path")

        if path == "Square":
            square_path(v, td)
        elif path == "Triangular":
            triangular_path(v, td)
        elif path[:4] == "Star":
            try:
                star_tips = int(path[4])
                star_path(v, td, star_tips)
            except:
                star_path(v, td, 5)
        elif path == "Rhombus":
            rhombus_path(v, td)
        else:
            print('Error: unknown movement')

    except rospy.ROSInterruptException:
        pass
