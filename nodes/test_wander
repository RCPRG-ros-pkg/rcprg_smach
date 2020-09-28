#!/usr/bin/env python
# encoding: utf8

import sys
import rospy

def main():
    rospy.init_node('rico_task_wander')
    rospy.sleep(0.5)
    print 'wander ENTER', sys.argv
    while not rospy.is_shutdown():
        print 'wander is active'
        rospy.sleep(2)
    print 'wander EXIT'

if __name__ == '__main__':
    main()
