#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
# import cv2

# green = np.uint8([[[0, 0, 0]]])
# hsvGreen = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
# print(hsvGreen)
# lowerLimit = (hsvGreen[0][0][0]-10,100,100)
# upperLimit = (hsvGreen[0][0][0]+10,255,255)
# print(upperLimit)
# print(lowerLimit)
def Sort_tile_mass_num(resultuv):
    tempxy = []
    tempy=[]
    resultmp=[]
    lastresult=[]
    tmp=[]
    newreslutuv=[]
    # print resultuv
    for i in range(len(resultuv)):
        if resultuv[i][0][0]==i+1:
            newreslutuv.append(resultuv[i])
    for i in newreslutuv:
        # if i[0][0]
        tmp.append(i[1])
    tempxy=sorted(tmp, reverse = True,key=lambda k: [k[1], k[0]])
    print tempxy
    for i in tempxy:
        for ii in newreslutuv:
            if i[0]==ii[1][0] and i[1]==ii[1][1]:
                resultmp.append(ii)
    for i in range(len(resultmp)):
        woqu=(resultmp[i][1:])
        # print "woqutype",type(woqu)
        woqu.insert(0,[i+1])
        # print woqu
        lastresult.append(woqu)

    # for i in resultuv:
    #     tempx.append(i[1][0])
    # print 'b',tempx
    # tempx = list(set(tempx))
    # print 'a',tempx
    # tempx.sort(reverse=True)
    # for j in tempx:  # 升序排列
    #     for ii in resultuv:
    #         if ii[1][0] == j:
    #             resultx.append(ii)
    #
    # for i in resultx:
    #     tempy.append(i[1][1])
    # tempy.sort(reverse=True)
    # tempy = list(set(tempy))
    # for j in tempy:  # 升序排列
    #     for ii in resultx:
    #         if ii[1][1] == j:
    #             lastresult.append(ii)
    # print tempx
    return lastresult
def main():
    kk= [[[1], [380, 427], [[415, 396], [411, 465], [348, 461], [346, 391]]], [[2], [379, 348], [[415, 381], [413, 313], [349, 383], [344, 314]]], [[3], [300, 422], [[334, 391], [332, 460], [269, 455], [267, 388]]], [[4], [299, 345], [[334, 313], [331, 382], [265, 311], [264, 377]]], [[5], [216, 420], [[254, 388], [246, 456], [186, 385], [185, 455]]], [[6], [219, 342], [[255, 309], [255, 375], [188, 378], [185, 310]]], [[7], [299, 265], [[334, 298], [333, 232], [265, 232], [265, 298]]], [[8], [220, 264], [[256, 299], [254, 231], [186, 298], [185, 232]]], [[9], [380, 266], [[417, 233], [414, 303], [346, 230], [344, 300]]], [[9], [380, 266], [[417, 234], [413, 303], [347, 230], [344, 296]]]]
    dd=Sort_tile_mass_num(kk)
    for u in kk:
        print 'kk',u
    for i in dd:
        print i
if __name__=="__main__":
    main()