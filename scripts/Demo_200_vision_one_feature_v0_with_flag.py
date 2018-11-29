#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy,math
import Quaternion as Q
import time
from numpy import linalg

import yaml
import os
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from tilling_robot.msg import uv
from tilling_robot.msg import tileuv
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import UInt16,Float64

from std_msgs.msg import String
from Functions_for_other_py import *

# from tilling_robot.msg import code_flags

from code_flags_sub import *
from std_msgs.msg import UInt8
"""
v2 tiling use one after another one,not using image space path planning
"""
"""
os.system("rostopic pub io_state std_msgs/String "55C8010155" --once")
"""
class TilingVisionControl():
    def __init__(self,nodename,urdfname,detat,lamda,califilename,camf,kappa=0.7,delta=5):
        self.nodename=nodename
        self.califilename=califilename
        self.urdfname=urdfname
        self.camf=camf
        self.detat=detat
        self.kappa=kappa
        self.delta=delta
        self.lamda=lamda

        self.tile_0_buf=[]
        self.tile_1_buf = []
        self.tile_2_buf =[]
        self.tile_3_buf = []
        self.ledstate=None
        self.changeuv=None
        self.w_pub = rospy.Publisher("/w_param", Float64, queue_size=10)
    def Init_node(self):
        rospy.init_node(self.nodename)
        # tile_reader = TileUvRead()
        tileuv_sub = rospy.Subscriber("/tilling_robot/tile_uv", tileuv, self.callback)
        ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        sucker_center_obj_pub = rospy.Publisher("tilling_robot/sucker_center_obj_uv", uv, queue_size=10)
        sucker_center_desire_pub = rospy.Publisher("tilling_robot/sucker_center_desire_uv", uv, queue_size=10)
        return ur_pub,sucker_center_obj_pub,sucker_center_desire_pub
    """
    0:o
    1:d
    """
    def add_tile_info(self,msg,bufname):
        if len(bufname) == 10:
            bufname = bufname[1:]
            tile_id = msg.tile_id
            cen_uv = msg.cen_uv
            f1th_uv = msg.f1th_uv
            s2th_uv = msg.s2th_uv
            t3th_uv = msg.t3th_uv
            f4th_uv = msg.f4th_uv
            bufname.append(
                [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
            # print "---------self.uvlist_buf",self.uvlist_buf
        else:
            tile_id = msg.tile_id
            cen_uv = msg.cen_uv
            f1th_uv = msg.f1th_uv
            s2th_uv = msg.s2th_uv
            t3th_uv = msg.t3th_uv
            f4th_uv = msg.f4th_uv
            bufname.append(
                [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])

    def callback(self, msg):
        if msg.tile_id == 1:
            self.add_tile_info(msg,self.tile_0_buf)
        else:
            # print "wait opencv get tile uv ----"
            # time.sleep(1)
            pass
        if msg.tile_id == 2:
            self.add_tile_info(msg,self.tile_1_buf)
        else:
            pass
            # print "wait opencv get tile uv ----"
            # time.sleep(1)
        if msg.tile_id == 3:
            self.add_tile_info(msg,self.tile_2_buf)
        else:
            # print "wait opencv get tile uv ----"
            # time.sleep(1)
            pass
        if msg.tile_id == 4:
            self.add_tile_info(msg,self.tile_3_buf)
        else:
            # print "wait opencv get tile uv ----"
            # time.sleep(1)
            pass

    def Get_ur_X(self,info):

        aa = Get_X_from_cali_quaternion(info)
        aa = numpy.mat(aa)
        # print "X", aa
        return aa.reshape((4, 4))
    def get_jacabian_from_joint(self,urdfname,jointq):
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # # print pose
        # #print list(pose)
        # q0=Kinematic(q)
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        # else:
        #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # # print "----------iverse-------------------\n", q_ik
        #
        # if q_ik is not None:
        #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        #     print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose
    def get_cam_data(self):
        f=open(self.califilename)
        yamldata=yaml.load(f)
        #print yamldata
        kx =  yamldata['camera_matrix']['data'][0]
        #print kx
        ky = yamldata['camera_matrix']['data'][4]
        #print ky
        u0=yamldata['camera_matrix']['data'][2]
        v0 = yamldata['camera_matrix']['data'][5]
        cam = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0}
        return cam
    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=self.camf#m
        kx = cam['kx']
        ky = cam['ky']
        arfx=kx/camf
        arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-arfx/z,0,uba/z,1/arfx*uba*vba,-(arfx**2+uba**2)/arfx,vba,0,-arfy/z,vba/z,(arfy**2+vba**2)/arfy,-uba*vba/arfx,-uba]]
        J=numpy.array(L).reshape((2,6))
        return J
    #uv more than one
    #,uv = [[672, 672], [632, 662]]
    def vis2jac_mt1(self,uvm,z):
        if len(uvm)>1:
            L=self.vis2jac(uvm[0],z)
            #L=numpy.array(L).reshape((2,6))
            for i in xrange(1,len(uvm)):
                J=numpy.row_stack((L,self.vis2jac(uvm[i],z)))
                #print "-------",i,J
                L=J
            #print "vision jacobian last\n",J
            return J
        else:
            return self.vis2jac(uvm[0],z)

    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        return kk.reshape((1,2))
    """
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    #get camera frame speed,you must change to ee frame
    #uvm means now uv
    """

    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lamda*numpy.dot(JJ,e.T)
        return vdot

    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q,info):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q)
        #2,get ee(AX=XB) to camera frame jacabian
        X=self.Get_ur_X(info)#numpu array
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        # print "cam_speed--------",cam_speed
        ee_speed_in_eeframe = numpy.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [0, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = numpy.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        # print "ee_speed-----before changing--------",ee_speed_in_base

        # print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed
    def get_deta_joint_angular(self,j_speed):
        #print j_speed
        joint_angular=float(self.detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular

    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        # print "list",detajoint.tolist()
        return listangular
    def check_vison_error_is_zero(self,desireuv,nowuv,error):
        feature_error=self.get_feature_error(desireuv,nowuv).reshape((1,2))
        print "feature_error",feature_error.tolist()
        if abs(feature_error.tolist()[0][0])<=error and abs(feature_error.tolist()[0][1])<=error:
            return 1
        else:
            print "Need more time to go to object----------"
            return 0
    def ibvs_run_ur5(self,uvm,z,q,info,xp):

        """
        First,Get the uv to drive ur5
        x=uvm[0]=nowuv

        """
        # xp=self.tile_0_buf[-1][1]
        """
        Second,caculating cam vodt and deta joint speed
        desireuv=xp
        """

        joint_speed_dot=self.get_joint_speed(uvm, z, xp, uvm[0], q, info)
        # print "joint_speed_dot",joint_speed_dot
        """
        Third,caculating deta joint speed
        """
        deta_joint_angular=self.get_deta_joint_angular(joint_speed_dot)
        # print "deta_joint_angular",deta_joint_angular
        """
        Fourth,get joint angular
        """
        pub_joint=self.get_joint_angular(q, deta_joint_angular)
        # print "pub_joint",pub_joint
        return pub_joint

    def Move_ur(self,q_pub_now,ace,vel,urt):
        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
            q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(
            ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(urt) + ")"
        print "ss",ss
        return ss
    def Open_sucking_close_IoBoard(self,flag):
        Protocol="55C8010"+str(flag)+"55"
        Pub_str='rostopic pub io_state std_msgs/String '+Protocol+' --once'
        os.system(Pub_str)


    """
    Low power is valid
    Before using this function,you must make sure,P13 is high.Led open
    """
    def Open_sucking_close_Ardunio(self):
        Protocol="{}"
        Pub_str='rostopic pub /toggle_led std_msgs/Empty '+Protocol+' --once'
        os.system(Pub_str)
    def move_sucker_to_tile(self,z_distance,tile_width,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, T[3], T[7], z_distance)  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    def move_ur_to_desire(self,x_distance,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, x_distance, T[7], T[11])  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    """
    这是一个非常大的坑，notice the base link ,where the line oretation is the -y in our tools link
    """
    def move_ur_to_desire_vertical(self,x_distance,y_distance,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, x_distance, y_distance, T[11])  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    def change_angle_to_pi(self,qangle):
        temp = []
        for i in xrange(len(qangle)):
            temp.append(qangle[i] / 180.0 * 3.14)
        return temp
    """
    uv0,初始状态下的uv,level,螺旋贴砖的层次
    """
    def Caculate_desire_uv_for_place(self,level):
        size=90
        uv0=self.tile_0_buf[-1][1]
        id1=[uv0[0]+3,uv0[1]+12]
        id2=[uv0[0],uv0[1]-size]
        id3=[uv0[0]-size,uv0[1]-size]
        id4=[uv0[0]-size+19,uv0[1]]
        id5=[uv0[0]-size+19,uv0[1]+size-16]
        # id4=[uv0[0]-size+10,uv0[1]-10]
        # id5=[uv0[0]-size+10,uv0[1]+size-10]
        # return [id1,id2,id3,id4,id5]
        return [id1, id4, id5,id2, id3]
    """
    基于joint q做旋转,末端做90度旋转,rotation angular
    """
    def Rotaion_tool_90(self,q_now,angular):
        q_new=[]
        for i in xrange(len(q_now)):
            if i ==5:
                q_new.append(q_now[i]+(angular*math.pi/180))
            else:
                q_new.append(q_now[i])
        return q_new
    def os_move_ur(self,q_pub_now,ace,vel,urt):
        movestring = str(q_pub_now[0]) + ',' + str(q_pub_now[1]) + ',' + str(q_pub_now[2]) + ',' + str(
            q_pub_now[3]) + ',' + str(q_pub_now[4]) + ',' + str(q_pub_now[5]) + '],a=' + str(ace) + ',' + 'v=' + str(
            vel) + ',' + 't=' + str(urt)
        pubstring = 'rostopic pub /ur_driver/URScript std_msgs/String ' + '"movej([' + movestring + ')" --once'
        tmp = os.popen(pubstring).readlines()
        print tmp
def main():
    #uvlist=[123.0,112.0]
    uvlist=[]
    camf=624.0429 * 1e-03
    #316,251
    # uvcentral_place=[316,251]
    uvcentral_place=[342,178]#320,201#180323,174#325,163#355,diyige canshuda,you yi
    uvcentral = [375,165]#sucking central109-125OK,370,201#368,95#120#362,123#133da xiang xia,
    First_joint_angular=[]
    calibinfo=[
        0.109982645426,
        0.114476746567,
        -0.0415924235801,
        0.249492772999,
        0.523487628443,
        0.239612281752,
        0.778652691205
    ]
    urdfname = "/data/ros/ur_ws_yue/src/tilling_robot/urdf/ur5.urdf"
    cailename = "/data/ros/ur_ws_yue/src/tilling_robot/yaml/cam_500_logitech.yaml"
    nodename="tilling_vision_control"
    tile_width = 0.01 #m
    ace_ibvs=50
    vel_ibvs=0.1
    ace=1.4
    vel=1.05
    urt=0
    detat=0.05
    ratet=30
    z_distance=0
    lamda=3.666666
    # lamda = 1000.066666
    z=0.45
    ur_reader = Urposition()
    code_flag_sub=Codeflags()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    u_error_pub = rospy.Publisher("/object_feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/object_feature_v_error", Float64, queue_size=10)
    sub5 = rospy.Subscriber("/pick_place_tile_vision/open_ur_rotation_id", UInt8,code_flag_sub.callback_open_ur_rotation_id)
    """
    Those two flag use to make sucker sucking tile
    """
    object_flag=0
    open_vison_flag= 0
    desire_flag=0
    turn_on_vision_flag=0
    """
    nodename,urdfname,delta,kappa,lamda,califilename,camf)
    """
    F0=TilingVisionControl(nodename,urdfname,detat,lamda,cailename,camf)
    ur_pub, sucker_center_obj_pub, sucker_center_desire_pub=F0.Init_node()
    rate = rospy.Rate(ratet)
    First_joint_angular = ur_reader.ave_ur_pose

    Object_joint_angular_vision_state=[]
    Object_joint_angular_sucking_state = [-58.85,-90.39,-88.02,-93.75,86.74,69.84]#66.02
    Object_joint_angular_sucking_state_new= F0.change_angle_to_pi(Object_joint_angular_sucking_state)
    Desire_joint_angular_vision_state=[]
    Desire_joint_angular_place_state = [-175.78,-78.37,-116.14,17.25,86.49,43.05]
    Desire_joint_angular_place_state_new = F0.change_angle_to_pi(Desire_joint_angular_place_state)
    print "First_joint_angular",First_joint_angular

    # tile_2=[-217.59,-105.73,-89.85,22.94,88.92,45.29]
    # tile_2_new=F0.change_angle_to_pi(tile_2)
    # tile_3=[-217.92,-104.96,-82.53,16.9,88.92,45.27]
    # tile_3_new=F0.change_angle_to_pi(tile_3)

    count_for_tile=0
    open_suking_flag=0
    open_roation_flag=0
    open_move_to_desire_flag=0
    open_vison_flag_desire=0
    open_suking_flag_desire=0
    open_move_to_object_flag=0
    open_align_line_flag=0
    close_align_line_flag=0
    open_vision_desire_flag=0
    open_vision_object_flag = 0
    tile_nums=0#为了记录吸第几块砖
    msg_uv_o=uv()
    msg_uv_d = uv()
    os.system("rostopic pub /pick_place_tile_vision/object_detect_id std_msgs/UInt8 '1' --once")
    while not rospy.is_shutdown():
        # print "F0.tile_0_buf",F0.tile_0_buf[-1]
        # print "F0.tile_1_buf",F0.tile_1_buf[-1]
        # print "F0.tile_2_buf",F0.tile_2_buf[-1]
        # print "F0.tile_3_buf",F0.tile_3_buf[-1]
        # time.sleep(1)
        desire_tile_path_cacu=[]
        try:
            """
            1,通过object_ibvs_id控制视觉伺服
            """
            """
            Go to object position,just need opreating tile 1,
            Now UV also is [316,251]
            """
            msg_uv_o.uvinfo=uvcentral
            msg_uv_d.uvinfo=uvcentral_place
            sucker_center_obj_pub.publish(msg_uv_o)
            sucker_center_desire_pub.publish(msg_uv_d)
            q_now = ur_reader.ave_ur_pose
            print "q_now",q_now
            if len(q_now) != 0:
                if len(F0.tile_0_buf)!=0 and open_vison_flag ==0:
                    print "First,Go to object tile position--------------"
                    uvm = [uvcentral]
                    xp=F0.tile_0_buf[-1][1]#目标位置
                    q_pub_now=F0.ibvs_run_ur5(uvm,z,q_now,calibinfo,xp)
                    MoveUrString=F0.Move_ur(q_pub_now, ace_ibvs, vel_ibvs, urt)
                    ur_pub.publish(MoveUrString)
                    time.sleep(0.05)
                    feature_error_zero_flag=F0.check_vison_error_is_zero(F0.tile_0_buf[-1][1],uvcentral,5)
                    """
                    if error is zero ,we close vision servo
                    """
                    if feature_error_zero_flag:
                        open_vison_flag =1
                        open_suking_flag=1
                    print "F0.tile_0_buf",F0.tile_0_buf[-1]
                """
                第二步，关闭视觉伺服，打开轨迹规划，并吸瓷砖
                """
                if open_vison_flag ==1 and open_suking_flag==1:
                    """
                    second,go to sucking tile
                    """
                    print "-------------num1,go to sucking tile------------"
                    z_distance=0.13
                    q_before_sucking=q_now
                    q_new_pub=F0.move_sucker_to_tile(z_distance,tile_width,q_now)
                    MoveUrString=F0.Move_ur(q_new_pub, ace, vel, urt)
                    ur_pub.publish(MoveUrString)
                    time.sleep(1.5)
                    print """num2,Open air pump for sucking"""
                    if open_suking_flag==1:
                        F0.Open_sucking_close_Ardunio()
                        print """num3,go back the same point"""
                        # print "-------Third,go back the same point---------"
                        MoveUrString=F0.Move_ur(q_before_sucking, ace, vel, urt)
                        ur_pub.publish(MoveUrString)
                        # time.sleep(1.5)
                        os.system("rostopic pub /pick_place_tile_vision/object_detect_id std_msgs/UInt8 '0' --once")
                    open_move_to_desire_flag=1
                """
                第三步，移动到目标位置
                """
                if open_move_to_desire_flag==1:
                    print """
                    Third,move ur to desire position.
                    """

                    MoveUrString=F0.Move_ur(Desire_joint_angular_place_state_new, ace, vel, urt)
                    ur_pub.publish(MoveUrString)
                    open_vison_flag = 1
                    open_suking_flag = 0
                    open_move_to_desire_flag=0
                    # open_vison_flag_desire=1
                    open_align_line_flag = 1
                    F0.tile_0_buf=[]
                    # time.sleep(3)
                    os.system("rostopic pub /pick_place_tile_vision/desire_detect_id std_msgs/UInt8 '1' --once")
                    time.sleep(0.1)
                    """
                    close all flags
                    """
                    """
                    打开瓷砖对齐程序
                    """
                if open_align_line_flag==1:
                    try:
                        #os.system("rostopic pub /pick_place_tile_vision/open_ur_rotation_id std_msgs/UInt8 '1' --once")
                        if len(code_flag_sub.open_ur_rotation_id_buf)!=0:
                            if code_flag_sub.open_ur_rotation_id_buf[-1]==0:

                                open_align_line_flag=0
                                open_vison_flag_desire =1
                                print "now tile is ok,you can go to next step-------"
                    except KeyboardInterrupt:
                        sys.exit()
                if len(F0.tile_0_buf) != 0 and open_vison_flag_desire == 1:
                    print """
                    第四步，打开目标空间的视觉伺服程序
                    """
                # if len(F0.tile_0_buf)!=0 and open_vison_flag_desire ==1:
                    uvcentral_place_2 = [316,251]
                    # uvm2=[uvcentral_place_2]
                    uvmm = [uvcentral_place]
                    print "num0，打开目标空间的视觉伺服程序"
                    print "-------目标空间贴第"+str(tile_nums)+"块砖-----"
                    xpp=F0.tile_0_buf[-1][1]

                    # uv0=xpp
                    # desire_tile_path_cacu=F0.Caculate_desire_uv_for_place(2)
                    # print "xpp=F0.tile_0_buf[-1][1]",F0.tile_0_buf[-1][1]
                    # print "desire_tile_path_cacu[tile_nums]",desire_tile_path_cacu[tile_nums]
                    # print "desire_tile_path_cacu----->>>>>", desire_tile_path_cacu
                    # time.sleep(1)
                    if tile_nums < 3:
                        q_pub_now_d=F0.ibvs_run_ur5(uvmm,z,q_now,calibinfo,xpp)
                        MoveUrString_1=F0.Move_ur(q_pub_now_d, ace_ibvs, vel_ibvs, urt)
                        ur_pub.publish(MoveUrString_1)
                        feature_error_zero_flag_d = F0.check_vison_error_is_zero(xpp,
                                                                                 uvcentral_place, 3)

                    """
                    if error is zero ,we close vision servo
                    """
                    if feature_error_zero_flag_d:
                        # open_suking_flag_desire = 1
                        open_vison_flag_desire =0
                        close_align_line_flag = 1
                        tile_nums+=1
                        time.sleep(0.5)
                        desire_tile_path_cacu=[]
                        print "---------you are now in desire space-----------"


                if open_vison_flag_desire ==0 and close_align_line_flag==1:
                    print """
                    第五步，关闭视觉伺服，打开轨迹规划，并释放瓷砖
                    """
                    print """
                    num0,go to sucking tile
                    """
                    q_before_sucking_d = q_now
                    print "tile_nums------>>>>>>>",tile_nums
                    x_distance = 0.65

                    # if (tile_nums-1)<3:

                    q_new_pub_d = F0.move_ur_to_desire(x_distance, q_before_sucking_d)
                    # q_after_sucking_d = q_new_pub_d
                    F0.os_move_ur(q_new_pub_d,ace, vel, urt)
                    # MoveUrString = F0.Move_ur(q_new_pub_d, ace, vel, urt)
                    # ur_pub.publish(MoveUrString)
                    # time.sleep(3)
                    open_suking_flag_desire=1
                    close_align_line_flag=0
                if open_suking_flag_desire==1:
                    print """
                    num1,close air pump for sucking
                    """
                    F0.Open_sucking_close_Ardunio()
                    open_suking_flag_desire=0
                    print """
                    num2,go back the same point
                    """
                    MoveUrString=F0.Move_ur(q_before_sucking_d, ace, vel, urt)
                    ur_pub.publish(MoveUrString)
                    time.sleep(1.5)
                    open_move_to_object_flag=1
                if open_move_to_object_flag==1:
                    print "#########Desire,last step,move ur to object position-------"
                    # x_distance = 0.35
                    # q_new_pub=F0.move_ur_to_desire(x_distance,q_now)
                    MoveUrString_2=F0.Move_ur(Object_joint_angular_sucking_state_new, ace, vel, urt)
                    ur_pub.publish(MoveUrString_2)
                    open_vison_flag = 0
                    open_suking_flag = 0
                    open_move_to_desire_flag=0
                    open_vison_flag_desire=0
                    open_move_to_object_flag=0
                    F0.tile_0_buf=[]
                    q_now=[]
                    # time.sleep(1)
                    os.system("rostopic pub /pick_place_tile_vision/desire_detect_id std_msgs/UInt8 '0' --once")
                    os.system("rostopic pub /pick_place_tile_vision/object_detect_id std_msgs/UInt8 '1' --once")
                    """
                    close all flags
                    """
                if tile_nums >= 3:
                    tile_nums = 0

            else:
                print "UR5 is Not Ok,Please check"
            rate.sleep()
        except KeyboardInterrupt:
            sys.exit()


if __name__=="__main__":
    main()

