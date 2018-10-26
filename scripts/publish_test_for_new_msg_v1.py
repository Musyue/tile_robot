#!/usr/bin/env python
import rospy
from tilling_robot.msg import code_flags
import yaml
class RuncommandFlag():
    def __init__(self):
        self.filepath="/data/ros/ur_ws_yue/src/tilling_robot/yaml/flags.yaml"
    def Init_node(self):
        rospy.init_node("Run_commands_node")
        pub = rospy.Publisher('/pick_place_tile_vision/pick_vision',code_flags,queue_size=10)
        return pub
    # def read_flags_from_yaml(self):
    #     f = open(self.filepath)
    #     datas = yaml.load(f)
    #     datas['flags']
    #     return
def main():

    R=RuncommandFlag()
    pub=R.Init_node()
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        msg=code_flags()
        msg.object_detect_id=1
        # msg.object_ibvs_id=2
        # msg.desire_detect_id=0
        # msg.desire_ibvs_id=4
        pub.publish(msg)
        rate.sleep()
if __name__=="__main__":
   main()