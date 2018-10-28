#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
rate=rospy.Rate(30)
def change_angle_to_pi(qangle):
    temp=[]
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
while not rospy.is_shutdown():
    t=0
    vel=0.3
    ace=50
    qq=[
        # [-94,-95.29,-90.44,-86.80,91.62,71.55]
        # [-68.91, -98.86, -81.19, -89.11, 89.35, 91.79]#for multitarget
        # [-110.18,-93.16,-92.82,-82.06,86.68,66.02]#pick
        [-219.61,-82.61,-128.57,31.76,86.31,42.58]#place
        # [-219.61, -82.61, -128.57, 31.76, 86.31, 131]
        # [-57.10,-59.27,-114.45,-91.76,91.41,71.59]First View

        ]
    for ii in xrange(len(qq)):
        qt=change_angle_to_pi(qq[ii])
        # time.sleep(1)
        # qt=[3.1897695779800417, -2.471768395100729, 1.8933079242706299, -2.4048668066607877, -3.1475941101657314, -3.8097620646106165]
        moveur(pub, qt,ace,vel,t)
        time.sleep(0.5)
    rate.sleep()
