#!/usr/bin/env python
# encoding: utf8

import sys
import rospy

def main():
    rospy.init_node('rico_task_move_to')
    rospy.sleep(0.5)
    print 'move_to ENTER', sys.argv
    counter = 20
    while not rospy.is_shutdown() and counter > 0:
        print 'move_to is active'
        rospy.sleep(1)
        counter -= 1
    print 'move_to EXIT'

if __name__ == '__main__':
    main()
