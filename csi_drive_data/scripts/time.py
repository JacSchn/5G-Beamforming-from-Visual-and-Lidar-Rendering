#!/usr/bin/env python3
import rospy

rospy.init_node('talker', anonymous=True)


while not rospy.is_shutdown():
    now = rospy.Time.now()
    print("%i.%i" % (now.secs, now.nsecs))
