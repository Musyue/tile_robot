#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
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
def movelur(pub,q,ace,vel,t):
    ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
def movecur(pub,q,ace,vel,t):
    ss="movec(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
#
def main():
    while not rospy.is_shutdown():
        print "start while-----"
        t=0
        # vel=0.1
        # ace=50
        vel=1.05
        ace=1.4
        qq=[#20181107
            # [-175.78,-87.78,-120.25,29.73,86.97,45.0]
            # [-175.78,-86.57,-120.76,29.08,86.95,43.05]
            # [-146.89,-100.45,-129.43,51.84,57.70,43.96]
            # [-94,-95.29,-90.44,-86.80,91.62,71.55]
            # [-68.91, -98.86, -81.19, -89.11, 89.35, 91.79]#for multitarget
            # [-58.85, -90.39, -88.02, -91.27, 90.11, 69.44]#new pick-58.85,-90.39,-88.02,-91.27,90.11,69.44
            # [-58.85,-90.39,-88.02,-93.75,86.74,69.84]#pick
            # [-175.78,-86.61,-120.25,29.08,87.00,43.05]#place
            [-175.78,-82.87,-112.83,18.08,86.98,43.05]#new place
            # [-175.78, -86.61, -120.25, 29.08, 87.00, -46.05]
            # [-216.24, -105.93, -88.34, 21.07, 86.30, 42.29]
            # [-215.68,-105.73,-91.15,22.74,89.30,45.29]
            # [- 212.39, -102.54, -81.26, 5.91, 77.85, 42.29]
            # [- 210.07, -101.68, -86.22, 8.10, 71.50, 42.29]
            # [-208.48, -101.54, -81.26, 6.41, 73.60, 42.29]
            #[]第二块[-208.54,-101.74,-89.76,15.78,69.71,42.29]
            #[]第三块[-208.48,-101.54,-81.26,6.13,69.22,42.29]
            # [-219.61, -82.61, -128.57, 31.76, 86.31, 131]
            # [-57.10,-59.27,-114.45,-91.76,91.41,71.59]First View

            ]
        for ii in xrange(len(qq)):
            qt=change_angle_to_pi(qq[ii])
            # qt=[-3.59860155164,-1.82648800883,-1.41735542252,0.0812199084238,1.27000134315,0.734254316924]
            # qt=[-3.66249992472,-1.27642603705,-1.9559700595,0.0701396996895,1.3338858418,0.73287290524]
            # time.sleep(1)
            # qt=[3.1897695779800417, -2.471768395100729, 1.8933079242706299, -2.4048668066607877, -3.1475941101657314, -3.8097620646106165]
            moveur(pub, qt,ace,vel,t)
            print "start "+str(ii)+" ur position-----"
            time.sleep(4)
        print "after while------"
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()