#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
import os, time
import sys
"""
os.system("rostopic pub /pick_place_tile_vision/object_ibvs_id std_msgs/UInt8 "1" --once")
"""
class Codeflags():

    def __init__(self, name = "ur_info_subscriber" ):

        self.name = name
        self.object_detect_id_buf = []
        self.object_ibvs_id_buf = []
        self.desire_detect_id_buf = []
        self.desire_ibvs_id_buf = []
        self.open_ur_rotation_id_buf = []
    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/pick_place_tile_vision/object_ibvs_id", UInt8, self.callback_object_ibvs)
        sub = rospy.Subscriber("/pick_place_tile_vision/object_detect_id", UInt8, self.callback_object_detect_id)
        sub = rospy.Subscriber("/pick_place_tile_vision/desire_detect_id", UInt8, self.callback_desire_detect_id)
        sub = rospy.Subscriber("/pick_place_tile_vision/desire_ibvs_id", UInt8, self.callback_desire_ibvs_id)
        sub = rospy.Subscriber("/pick_place_tile_vision/open_ur_rotation_id", UInt8, self.callback_open_ur_rotation_id)
        return sub
    def callback_object_ibvs(self,msg):
        """
        object_ibvs_id_buf
        """
        if len(self.object_ibvs_id_buf)==10:
            self.object_ibvs_id_buf=self.object_ibvs_id_buf[1:]
            self.object_ibvs_id_buf.append(msg.data)
        else:
            self.object_ibvs_id_buf.append(msg.data)
    def callback_desire_ibvs_id(self,msg):
        """
        desire_ibvs_id_buf
        """
        if len(self.desire_ibvs_id_buf)==10:
            self.desire_ibvs_id_buf=self.desire_ibvs_id_buf[1:]
            self.desire_ibvs_id_buf.append(msg.data)
        else:
            self.desire_ibvs_id_buf.append(msg.data)
    def callback_object_detect_id(self,msg):
        """
        object_detect_id_buf
        """
        if len(self.object_detect_id_buf)==10:
            self.object_detect_id_buf=self.object_detect_id_buf[1:]
            self.object_detect_id_buf.append(msg.data)
        else:
            self.object_detect_id_buf.append(msg.data)

    def callback_desire_detect_id(self, msg):
        """
        desire_detect_id_buf
        """
        if len(self.desire_detect_id_buf)==10:
            self.desire_detect_id_buf=self.desire_detect_id_buf[1:]
            self.desire_detect_id_buf.append(msg.data)
        else:
            self.desire_detect_id_buf.append(msg.data)
    def callback_open_ur_rotation_id(self,msg):
        """
        open ur5 tool link rotaion for paralling tile width
        """
        if len(self.open_ur_rotation_id_buf)==10:
            self.open_ur_rotation_id_buf=self.open_ur_rotation_id_buf[1:]
            self.open_ur_rotation_id_buf.append(msg.data)
        else:
            self.open_ur_rotation_id_buf.append(msg.data)

def main():
    code_flag_info_reader = Codeflags()
    code_flag_info_reader.Init_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print "object_detect_id_buf",code_flag_info_reader.object_detect_id_buf
        print "desire_ibvs_id_buf",code_flag_info_reader.desire_ibvs_id_buf
        print "object_ibvs_id_buf",code_flag_info_reader.object_ibvs_id_buf
        print "desire_detect_id_buf",code_flag_info_reader.desire_detect_id_buf
        print "open_ur_rotation_id_buf",code_flag_info_reader.open_ur_rotation_id_buf

        # print "buf",code_flag_info_reader.code_change_flag_buf
        # print ( "now_pos: ", type(ur_info_reader.now_ur_pos))

        # print ("ave_pos_ur:", ur_info_reader.ave_ur_pose)
    # rospy.spin()
        rate.sleep()


if __name__ == "__main__":
    main()