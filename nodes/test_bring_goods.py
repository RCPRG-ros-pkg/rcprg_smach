#!/usr/bin/env python
# encoding: utf8

import sys
import rospy

def main():
    rospy.init_node('rico_task_bring_goods')
    rospy.sleep(0.5)
    print 'bring_goods ENTER', sys.argv
    counter = 20
    while not rospy.is_shutdown() and counter > 0:
        print 'bring_goods is active'
        rospy.sleep(1)
        counter -= 1
    print 'bring_goods EXIT'

if __name__ == '__main__':
    main()
