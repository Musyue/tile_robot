#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from ur5_kinematics import *
import numpy,math
import time,os
"""
rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
class TilingKinematics():
    def __init__(self):
        pass
    def Init_node(self):
        rospy.init_node("tile_just_kinematics")
        pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        arduniopub = rospy.Publisher("/toggle_led", Empty, queue_size=10)
        return arduniopub,pub

    def moveur(self,q, ace, vel, t):
        ss = "movej([" + str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "," + str(q[3]) + "," + str(q[4]) + "," + str(
            q[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        # print ss
        # pub.publish(ss)
        return ss

    def get_T_translation(self,T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    """
    T.translation is the marker coordinate in base frame,T.Rotaion is the posture in base frame
    level is the layer of tiles,such as 2x2,level=2,3x3,level=3
    """
    def caculating_path_to_pose(self,level,T):
        trans0=[T[3],T[7],T[11]]
        x0=T[3]
        y0=T[7]
        z0=T[11]
        result=[]
        for i in xrange(level):
            if i % 2==0 and i != 0:#even number
                for ii in xrange(level):
                    result.append([x0+0.148,y0-0.06*ii,z0+0.12*(i-1)])
            elif i%2 !=0:#odd number
                for ii in range(level)[::-1]:
                    result.append([x0+0.148,y0-0.06*ii,z0+0.06*i])
            else:
                for ii in xrange(level):
                    result.append([x0+0.148,y0-0.06*ii,z0])
        # print result
        all_result=[]

        for i in xrange(len(result)):
            temp = []
            print "level"+str(i)+"layer"+str(result[i])
            for j in xrange(len(T)):

                if j==3:
                    temp.append(result[i][0])
                elif j==7:
                    temp.append(result[i][1])
                elif j==11:
                    temp.append(result[i][2])
                else:
                    temp.append(T[j])
            all_result.append(temp)

        return all_result
    def caculating_path_to_pose_1(self,level,T):
        trans0=[T[3],T[7],T[11]]
        x0=T[3]
        y0=T[7]
        z0=T[11]
        result=[]
        for i in xrange(level):
            if i % 2==0 and i != 0:#even number
                for ii in xrange(level):
                    if ii==1:
                        result.append([x0,y0-0.047*ii+0.0035,z0+0.003+0.1*(i-1)])
                    elif ii==0:
                        result.append([x0, y0 - 0.047 * ii+0.0035, z0+0.1*(i-1)])
                    elif ii==3:
                        result.append([x0, y0 - 0.0475 * ii+0.0035, z0+0.1*(i-1)+0.0071])
                    else:
                        result.append([x0, y0 - 0.0475 * ii+0.0035, z0+0.1*(i-1)+0.0048])

                    # result.append([x0,y0-0.05*ii,z0+0.12*(i-1)])
            # elif i%2 !=0:#odd number
            elif i ==1:  # odd number
                for ii in range(level):
                    if ii==1:
                        result.append([x0,y0-0.047*ii+0.003,z0+0.003+0.05*i])
                    elif ii==0:
                        result.append([x0, y0 - 0.047 * ii+0.003, z0+0.05*i])
                    elif ii==3:
                        result.append([x0, y0 - 0.0475 * ii+0.003, z0+0.05*i+0.0071])
                    else:
                        result.append([x0, y0 - 0.047 * ii+0.003, z0+0.05*i+0.0048])
            elif i ==3:  # odd number
                for ii in range(level):
                    if ii == 1:
                        result.append([x0, y0 - 0.047 * ii + 0.0035, z0 + 0.003 + 0.05 * i])
                    elif ii == 0:
                        result.append([x0, y0 - 0.047 * ii + 0.0035, z0 + 0.05 * i])
                    elif ii == 3:
                        result.append([x0, y0 - 0.0475 * ii + 0.0035, z0 + 0.05 * i + 0.0071])
                    else:
                        result.append([x0, y0 - 0.047 * ii + 0.0035, z0 + 0.05 * i + 0.0048])
                    # result.append([x0,y0-0.05*ii,z0+0.06*i])
            else:
                for ii in xrange(level):

                    if ii==1:
                        result.append([x0,y0-0.047*ii,z0+0.003])
                    elif ii==0:
                        result.append([x0, y0 - 0.047 * ii, z0])
                    elif ii==3:
                        result.append([x0, y0 - 0.0475 * ii, z0+0.0071])
                    else:
                        result.append([x0, y0 - 0.047 * ii, z0+0.0048])

        # print result
        all_result=[]

        for i in xrange(len(result)):
            temp = []
            print "level"+str(i)+"layer"+str(result[i])
            for j in xrange(len(T)):

                if j==3:
                    temp.append(result[i][0])
                elif j==7:
                    temp.append(result[i][1])
                elif j==11:
                    temp.append(result[i][2])
                else:
                    temp.append(T[j])
            all_result.append(temp)

        return all_result
    """
    ur0:kinematics attribute
    """
    def get_IK_from_T(self,ur0, T, q_last):
        weights = [1.] * 6
        return ur0.best_sol(weights, q_last, T)

    def insert_new_xy(self,T, nx, ny, nz):
        temp = []
        for i in xrange(16):
            if i == 3:
                temp.append(nx)
            elif i == 7:
                temp.append(ny)
            elif i == 11:
                temp.append(nz)
            else:
                temp.append(T[i])
        return temp

    def change_angle_to_pi(self,qangle):
        temp=[]
        for i in xrange(len(qangle)):
            temp.append(qangle[i]/180.0*math.pi)
        return temp
    # def cacu_all_joint_path(self):
    """
    allpath: tiling path planning
    """
    def caculate_now_tile_to_wall(self,nowq,ur0):
        To=ur0.Forward(nowq)
        x0=To[3]
        y0=To[7]
        z0=To[11]
        newTo=self.insert_new_xy(To,x0+0.151,y0,z0)
        suckerq=self.get_IK_from_T(ur0,newTo,nowq)
        return suckerq
    def object_sucker(self,q,ur0):

        To=ur0.Forward(q)
        x0=To[3]
        y0=To[7]
        z0=To[11]
        newTo=self.insert_new_xy(To,x0,y0,z0-0.1)
        suckerq=self.get_IK_from_T(ur0,newTo,q)
        return suckerq
"""

        # print T0
        # arduniopub.publish(Empty())
        # time.sleep(0.5)
        # arduniopub.publish(Empty())
"""
def main():
    tilekinematic=TilingKinematics()
    arduniopub,urpub=tilekinematic.Init_node()
    # ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)
    urt = 0
    # vel=0.1
    # ace=50
    ace = 1.4
    vel = 1.05
    weights = [1.] * 6
    #desirejoint = [-164.77, -95.48, -85.66, 3.99, 79.48, 136.22]#

    """
    desire joint
    """
    desirejoint=[-156.66219892170741, -100.19231085664684, -102.11325784750225, 25.418514845256603, 71.31263635419347, 146.15356948265926]#-164.79,-99.49,-103.06,25.48,79.39,150.17
    newdesirejoint = tilekinematic.change_angle_to_pi(desirejoint)
    # print newdesirejoint
    """
    tiling joint
    """

    """
    object joint
    """
    pickjoint = [-102.01709971229116, -152.2762680782646, -41.79267441896641, -101.0759078408904, 127.12725382877485, 113.47118807019129]
    newpickjoint = tilekinematic.change_angle_to_pi(pickjoint)

    """
    sucking joint
    """
    picksuckerq=[-102.0214846937383, -153.09503290690637, -42.569616633282365, -99.47109752180687, 127.13487495385074, 113.47654020642491]
    newpicksuckerq = tilekinematic.change_angle_to_pi(picksuckerq)
    # print newpickjoint
    # print newpicksuckerq
    urkinematics = Kinematic()

    T0 = urkinematics.Forward(newdesirejoint)
    # print numpy.array(T0).reshape(4,4)
    path = tilekinematic.caculating_path_to_pose_1(4, T0)
    all_path =[]
    # print path
    temp=newdesirejoint
    for i in path:
        # print i
        print "------------------"
        kk=tilekinematic.get_IK_from_T(urkinematics,i,temp)
        all_path.append(kk.tolist())
        # print kk.tolist()
        temp=kk.tolist()
    print len(all_path)
    # print T0
    # T0 = urkinematics.Forward(newpickjoint)
    # print T0
    rate=rospy.Rate(1)
    count_tile=0
    sucking_flags=0
    count_main=0
    while not rospy.is_shutdown():
        # arduniopub.publish(Empty())
        # time.sleep(1)
        if sucking_flags==0:


            # time.sleep(1.5)
            if count_main>=1:#di er ge loop caihui publish
                # sucking_flags=1
                mvstring = tilekinematic.moveur(newpicksuckerq, 0.1, vel, urt)
                urpub.publish(mvstring)
                arduniopub.publish(Empty())
                time.sleep(2.0)
                mvstring1 = tilekinematic.moveur(newdesirejoint, ace, vel, urt)
                urpub.publish(mvstring1)
                time.sleep(3.)
                print "tile----------------->"+str(count_tile),all_path[count_tile]
                mvstring1 = tilekinematic.moveur(all_path[count_tile], ace, vel, urt)#tiling
                urpub.publish(mvstring1)
                time.sleep(1.)
                desiresuckerq=tilekinematic.caculate_now_tile_to_wall(all_path[count_tile],urkinematics)
                mvstring1 = tilekinematic.moveur(desiresuckerq, ace, vel, urt)#tiling
                urpub.publish(mvstring1)
                time.sleep(1.5)
                arduniopub.publish(Empty())
                time.sleep(0.8)
                # os.system("rostopic pub /toggle_led std_msgs/Empty '{}' --once")
                mvstring1 = tilekinematic.moveur(all_path[count_tile], ace, vel, urt)#back some z aix
                urpub.publish(mvstring1)
                time.sleep(1.5)
                mvstring1 = tilekinematic.moveur(newpickjoint, ace, vel, urt)#back object
                urpub.publish(mvstring1)
                time.sleep(3.0)
                count_tile+=1
                if count_tile==16:
                    sucking_flags=1
                    print "tile all tiles-------"
            # print count_main
        else:
            print "over----",count_main
        count_main+=1
        # if count_tile<len(all_path):
        #     print "count_tile", count_tile
        #     mvstring=tilekinematic.moveur(all_path[count_tile],ace,vel,urt)
        #     print mvstring
        #     urpub.publish(mvstring)
        #     time.sleep(1.5)
        #     count_tile+=1
        #
        # else:
        #     print "tiling is over------"
        rate.sleep()
if __name__ == "__main__":
    main()
