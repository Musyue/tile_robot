#!/usr/bin/env python
import rospy
from tilling_robot.msg import code_flags
import os, time
import sys

class Codeflag():

    def __init__(self, name = "ur_info_subscriber" ):

        self.name = name
        self.object_detect_id_buf = []
        self.object_ibvs_id_buf = []
        self.desire_detect_id_buf = []
        self.desire_ibvs_id_buf = []
    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/pick_place_tile_vision/pick_vision", code_flags, self.callback_vision)
        return sub
    # def add_buf(self,buf_list):
    #     if len(self.code_change_flag_buf)==10:
    #         self.code_change_flag_buf=self.code_change_flag_buf[1:]
    #         self.code_change_flag_buf.append(msg.data)
    #     else:
    #         self.code_change_flag_buf.append(msg.data)
    def callback_vision(self, msg):
        # print "msg.object_detect_id",msg.object_detect_id
        # print "msg.object_ibvs_id",msg.object_ibvs_id
        # print "msg.desire_detect_id",msg.desire_detect_id
        # print "msg.desire_ibvs_id",msg.desire_ibvs_id
        """
        object_detect_id_buf
        """
        if len(self.object_detect_id_buf)==10:
            self.object_detect_id_buf=self.object_detect_id_buf[1:]
            self.object_detect_id_buf.append(msg.object_detect_id)
        else:
            self.object_detect_id_buf.append(msg.object_detect_id)
        """
        object_ibvs_id_buf
        """
        if len(self.object_ibvs_id_buf)==10:
            self.object_ibvs_id_buf=self.object_ibvs_id_buf[1:]
            self.object_ibvs_id_buf.append(msg.object_ibvs_id)
        else:
            self.object_ibvs_id_buf.append(msg.object_ibvs_id)
        """
        desire_detect_id_buf
        """
        if len(self.desire_detect_id_buf)==10:
            self.desire_detect_id_buf=self.desire_detect_id_buf[1:]
            self.desire_detect_id_buf.append(msg.desire_detect_id)
        else:
            self.desire_detect_id_buf.append(msg.desire_detect_id)
        """
        desire_ibvs_id_buf
        """
        if len(self.desire_ibvs_id_buf)==10:
            self.desire_ibvs_id_buf=self.desire_ibvs_id_buf[1:]
            self.desire_ibvs_id_buf.append(msg.desire_ibvs_id)
        else:
            self.desire_ibvs_id_buf.append(msg.desire_ibvs_id)

def main():
    code_flag_info_reader = Codeflag()
    code_flag_info_reader.Init_node()
    while not rospy.is_shutdown():
        print "object_detect_id_buf",code_flag_info_reader.object_detect_id_buf
        print "desire_ibvs_id_buf",code_flag_info_reader.desire_ibvs_id_buf
        print "object_ibvs_id_buf",code_flag_info_reader.object_ibvs_id_buf
        print "desire_detect_id_buf",code_flag_info_reader.desire_detect_id_buf
        # print "buf",code_flag_info_reader.code_change_flag_buf
        # print ( "now_pos: ", type(ur_info_reader.now_ur_pos))

        # print ("ave_pos_ur:", ur_info_reader.ave_ur_pose)
    # rospy.spin()


if __name__ == "__main__":
    main()