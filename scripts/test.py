#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os,sys
# o_path = os.getcwd()
# # print o_path
sys.path.append('..')
from DemoScripts import tiling_vision_control_one_feature_demo_v0
def Sort_tile_feature(approx):
    result = []
    new_approx = []

    for i in approx:
        new_approx.append(i[0])
    print new_approx
    temp = []
    for i in new_approx:
        temp.append(i[0])
    temp.sort(reverse=True)
    for i in temp:  # 升序排列
        for ii in new_approx:
            if ii[0] == i:
                result.append(ii)

    print result
approx=[[[342, 191]], [[415, 194]], [[406, 277]], [[334, 274]]]
Sort_tile_feature(approx)

# os.system("bash /data/ros/ur_ws_yue/src/tilling_robot/scripts/first_run.sh")