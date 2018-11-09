#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os,sys,math
# o_path = os.getcwd()
# # print o_path
sys.path.append('..')
# from DemoScripts import tiling_vision_control_one_feature_demo_v0
# def Sort_tile_feature(approx):
#     result = []
#     new_approx = []
#
#     for i in approx:
#         new_approx.append(i[0])
#     print new_approx
#     temp = []
#     for i in new_approx:
#         temp.append(i[0])
#     temp.sort(reverse=True)
#     for i in temp:  # 升序排列
#         for ii in new_approx:
#             if ii[0] == i:
#                 result.append(ii)
#
#     print result
# 1,add detecting 24 tiles
# 2,add 2x2 demo
# approx=[[[342, 191]], [[415, 194]], [[406, 277]], [[334, 274]]]
# Sort_tile_feature(approx)
q=[-3.06638444444,-1.51086333333,-2.09769444444,0.507284444444,1.51766666667,0.750983333333]
def Rotaion_tool_90(q_now,angular):
    q_new=[]
    for i in xrange(len(q_now)):
        if i ==5:
            q_new.append(q_now[i]+(angular*math.pi/180))
        else:
            q_new.append(q_now[i])
    return q_new
print "niubi",Rotaion_tool_90(q,-90)
# os.system("bash /data/ros/ur_ws_yue/src/tilling_robot/scripts/first_run.sh")