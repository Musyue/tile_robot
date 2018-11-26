#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from tilling_robot.msg import code_flags
from tilling_robot.msg import sucker_tile_line

from tilling_robot.msg import uv
from tilling_robot.msg import tileuv
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from code_flags_sub import *
from std_msgs.msg import UInt8

"""
先旋转最后一个关节，直到线检测斜率一样，斜率和目标直线反号，则减小最后一个关节的选择角度，朝负方向走。同号则朝正方向走
"""
import time
class MoveURToolParalleTile():
    def __init__(self):
        self.sucker_tile_buf=[]
        self.tile_uv_buf=[]
    def Init_node(self):
        rospy.init_node("move_ur5_by_test")
        pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
        tileuv_sub_sucker = rospy.Subscriber("/tilling_robot/sucker_line_uv", sucker_tile_line, self.callback_sucker)
        tileuv_sub = rospy.Subscriber("/tilling_robot/tile_uv", tileuv, self.callback_tile)
        return pub

    def callback_sucker(self,msg):

        if len(self.sucker_tile_buf)==10:
            self.sucker_tile_buf=self.sucker_tile_buf[1:]
            self.sucker_tile_buf.append([msg.sucker_tile_uv0.uvinfo,msg.sucker_tile_uv1.uvinfo,(msg.sucker_tile_slope,msg.sucker_tile_intercept)])
        else:
            self.sucker_tile_buf.append([msg.sucker_tile_uv0.uvinfo,msg.sucker_tile_uv1.uvinfo,(msg.sucker_tile_slope,msg.sucker_tile_intercept)])


    def callback_tile(self,msg):
        if msg.tile_id==1:
            tile_id = msg.tile_id
            cen_uv = msg.cen_uv.uvinfo
            f1th_uv = msg.f1th_uv.uvinfo
            s2th_uv = msg.s2th_uv.uvinfo
            t3th_uv = msg.t3th_uv.uvinfo
            f4th_uv = msg.f4th_uv.uvinfo
            # print "f1th_uv",f1th_uv
            # print "s2th_uv", s2th_uv
            if len(self.tile_uv_buf) == 10:
                self.tile_uv_buf = self.tile_uv_buf[1:]
                self.tile_uv_buf.append([tile_id, cen_uv, f1th_uv, s2th_uv, t3th_uv, f4th_uv])
            else:
                self.tile_uv_buf.append([tile_id, cen_uv, f1th_uv, s2th_uv, t3th_uv, f4th_uv])

    def change_angle_to_pi(self,qangle):
        temp = []
        for i in xrange(len(qangle)):
            temp.append(qangle[i] / 180.0 * 3.14)
        return temp

    def moveur(self,pub, q, ace, vel, t):
        ss = "movej([" + str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "," + str(q[3]) + "," + str(q[4]) + "," + str(
            q[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print ss
        pub.publish(ss)
    def caculate_tile_slop_intercept(self,tile_id):
        if len(self.tile_uv_buf)!=0:
            if self.tile_uv_buf[-1][0]==tile_id:
                """
                Point 0
                """
                x1=self.tile_uv_buf[-1][2][0]
                y1=self.tile_uv_buf[-1][2][1]
                """
                Point1
                """
                x2=self.tile_uv_buf[-1][3][0]
                y2 = self.tile_uv_buf[-1][3][1]
                """
                ((x1+x2)/2,(y1+y2)/2)
                """
                x3=(x1+x2)/2
                y3=(y1+y2)/2

                """
                central_point
                """
                x4=self.tile_uv_buf[-1][1][0]
                y4=self.tile_uv_buf[-1][1][1]
                if (x1-x2)!=0:
                    if x3-x4 !=0:
                        k1=0.1*(y3-y4)/(x3-x4)
                        intercept1=y4-k1*x4
                        return (k1,intercept1)
                    else:
                        k1=0
                        intercept1=y4
                        return (k1, intercept1)
                else:
                    pass
    def compare_slop_with_sucker_tile(self,tile_id):
        place_tile=self.caculate_tile_slop_intercept(tile_id)
        error_size=0.016#0.099
        try:
            if len(self.sucker_tile_buf)!=0:
                sucker_tile=self.sucker_tile_buf[-1][2]
                print "place_tile[0]-sucker_tile[0]", place_tile[0] - sucker_tile[0]
                print "place_tile[0]",place_tile[0]
                print "sucker_tile[0]",sucker_tile[0]
                if abs((place_tile[0]) - (sucker_tile[0])) <= error_size:
                    return 1
                else:
                    return 0
                # if place_tile[0]>0 and sucker_tile[0]>0:
                #     if abs((place_tile[0]) - (sucker_tile[0])) <= error_size:
                #         return 1
                #     else:
                #         return 0
                # elif place_tile[0]>0 and sucker_tile[0]<0:
                #     if abs((place_tile[0]) + (sucker_tile[0])) <= error_size:
                #         return 1
                #     else:
                #         return 0
                # elif place_tile[0]<0 and sucker_tile[0]>0:
                #     if abs((place_tile[0]) + (sucker_tile[0])) <= error_size:
                #         return 1
                #     else:
                #         return 0
                # elif place_tile[0]<0 and sucker_tile[0]<0:
                #     if abs(abs(place_tile[0]) - abs(sucker_tile[0])) <= error_size:
                #         return 1
                #     else:
                #         return 0
                # elif place_tile[0]==0 and sucker_tile[0]==0:
                #     return 1
                # elif (place_tile[0]==0 and sucker_tile[0]!=0) or (place_tile[0]!=0 and sucker_tile[0]==0):
                #     if abs(abs(place_tile[0]) - abs(sucker_tile[0])) <= error_size:
                #         return 1
                #     else:
                #         return 0
                # else:
                #     pass
        except:
            print "No slop"
    def judge_slop_with_sucker_tile_symbol(self,tile_id):
        place_tile=self.caculate_tile_slop_intercept(tile_id)
        if len(self.sucker_tile_buf)!=0:

            sucker_tile=self.sucker_tile_buf[-1][2]
            print "place_tile[0]",place_tile[0]
            print "sucker_tile[0]",sucker_tile[0]

            if place_tile[0]*sucker_tile[0]<=0:
                return 1
            else:
                return 0
    """
    rotation_flag>0,slop has the same symbol
    rotation_flag<0,slop has different symbol
    """
    def Just_move_tool_link(self,rotation_flag,q_now,step_size):
        q_new=[]
        if rotation_flag>0:
            for i in xrange(len(q_now)):
                if i==5:
                    temp=q_now[5]+step_size
                    q_new.append(temp)
                else:
                    q_new.append(q_now[i])
            return q_new
        else:
            for i in xrange(len(q_now)):
                if i==5:
                    temp=q_now[5]-step_size
                    q_new.append(temp)
                else:
                    q_new.append(q_now[i])
            return q_new


def main():
    t = 0
    # vel = 0.1
    # ace = 50
    vel = 1.4
    ace = 1.05
    q_now_start=[-110.25,-124.55,-46.95,-96.37,87.76,0.07]

    T=MoveURToolParalleTile()
    pub=T.Init_node()
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    q_now_new = T.change_angle_to_pi(q_now_start)
    step_size=0
    rate = rospy.Rate(10)
    close_roation_flag=0
    code_flag_sub = Codeflags()
    sub = rospy.Subscriber("/pick_place_tile_vision/open_ur_rotation_id", UInt8, code_flag_sub.callback_open_ur_rotation_id)

    temp_q=[]
    while not rospy.is_shutdown():
        q_now_new = ur_reader.ave_ur_pose
        print "q_now_new", q_now_new
        if len(code_flag_sub.open_ur_rotation_id_buf)!=0:
            if code_flag_sub.open_ur_rotation_id_buf[-1]==1:
                if (q_now_new[5]+step_size)<0.99:#0.95
                    print "q_now_new[5]",q_now_new[5]
                    if len(T.tile_uv_buf)!=0 and len(T.sucker_tile_buf)!=0:
                        # print "T.tile_uv_buf",T.tile_uv_buf
                        try:
                            if T.compare_slop_with_sucker_tile(1):
                                print "congrtuations tile is parallel"
                                # T.moveur(pub, q_now_new, ace, 0.3, t)
                                os.system("rostopic pub /pick_place_tile_vision/open_ur_rotation_id std_msgs/UInt8 '0' --once")
                                T.moveur(pub, q_now_new, ace, 0.3, t)
                                ##open second vision
                                # os.system(
                                #     "rostopic pub /pick_place_tile_vision/desire_ibvs_id std_msgs/UInt8 '1' --once")
                                # close_roation_flag=1
                            else:
                                if close_roation_flag==0:
                                    if T.judge_slop_with_sucker_tile_symbol(1):#异号
                                        pub_q=T.Just_move_tool_link(1,q_now_new,step_size)#rad,approxmate 6度
                                        print "The different symbol"
                                        # print "q_now_new",q_now_new
                                        # print "pub_q",pub_q
                                        T.moveur(pub,pub_q,ace,vel,t)
                                        # temp_q=pub_q
                                    else:#同号
                                        pub_q = T.Just_move_tool_link(-1, q_now_new, step_size)  # rad,approxmate 6度
                                        print "The same symbol"
                                        # print "q_now_new",q_now_new
                                        # print "pub_q",pub_q
                                        T.moveur(pub, pub_q, ace, vel, t)
                                        # temp_q = pub_q
                                    step_size+=0.05
                        except:
                            print "error"
                else:
                    step_size=0
            else:
                close_roation_flag = 0
                print "Please wait other process sequence---"
        else:
            print "Please wait open flags---"

        rate.sleep()
if __name__=="__main__":
    main()