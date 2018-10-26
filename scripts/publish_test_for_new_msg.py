#!/usr/bin/env python
import rospy
from tilling_robot.msg import code_flags
import yaml
rospy.init_node("code_msg_test")
pub = rospy.Publisher('/pick_place_tile_vision/pick_vision',code_flags,queue_size=10)

rate =rospy.Rate(2)
while not rospy.is_shutdown():
    msg=code_flags()
    msg.object_detect_id=1
    # msg.object_ibvs_id=2
    # msg.desire_detect_id=0
    # msg.desire_ibvs_id=4
    pub.publish(msg)
    rate.sleep()