#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from ur5_kinematics import *
import numpy
import time
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
rate=rospy.Rate(1)
def change_angle_to_pi(qangle):
    temp=[]
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
    # T is numpy.array
def get_T_translation(T):
    trans_x = T[3]
    trans_y = T[7]
    trans_z = T[11]
    return [trans_x, trans_y, trans_z]
def get_IK_from_T(ur0,T,q_last):
    weights = [1.] * 6
    return ur0.best_sol(weights,q_last,T)
def insert_new_xy(T,nx,ny,nz):
    temp=[]
    for i in xrange(16):
        if i==3:
            temp.append(nx)
        elif i==7:
            temp.append(ny)
        elif i==11:
            temp.append(nz)
        else:
            temp.append(T[i])
    return temp
while not rospy.is_shutdown():
    t=0
    vel=0.1
    ace=50
    qq=[-94,-95.29,-90.44,-86.80,91.62,71.55]
    Kine=Kinematic()
    qt = change_angle_to_pi(qq)
    T=Kine.Forward(qt)
    # moveur(pub, qt,ace,vel,t)
    new_T=insert_new_xy(T,T[3],T[7],0.28)#解算出来的是基于world的

    print "T",numpy.array(T).reshape(4,4)
    print "New",numpy.array(new_T).reshape(4,4)
    q_new=get_IK_from_T(Kine, new_T, qt)
    print "qt", qt
    # moveur(pub, qt, ace, vel, t)
    print "q_new",q_new.tolist()
    moveur(pub, q_new, ace, vel, t)
    rate.sleep()
