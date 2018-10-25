#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy
import Quaternion as Q
"""
This code need three relating code:
Quaternion.py
transfer.py
utlility.py
"""


def getpi(listb):
    listcc = []
    for i in listb:
        temp = i / 180 * 3.14
        listcc.append(temp)
    return listcc
def From_matrix_to_list(data):
    temp = data.tolist()
    result = []
    for i in xrange(len(temp)):
        for ii in xrange(len(temp)):
            result.append(temp[i][ii])
    return result
def Get_X_from_cali_quaternion(calibinfo):
    transition_L = numpy.array(calibinfo[:3]).T
    # print transition_L
    rot = calibinfo[3:6]
    s = calibinfo[6]
    q0 = Q.quaternion(s, numpy.mat(rot))
    # print "q02R--------\n", q0.r()
    # print q0.r()
    T_part1 = numpy.column_stack((q0.r(), transition_L))
    # print T_part1
    T_part2 = numpy.array([0, 0, 0, 1])
    # print T_part2
    T = numpy.row_stack((T_part1, T_part2))
    # print T
    T = T.tolist()
    T = T[0] + T[1] + T[2] + T[3]
    # print("T:" , T)
    return T
"""
AX=XB
    # calibration_info = [
    #translation: 
      x: 0.134484285561
      y: 0.0795027844047
      z: 0.110225634787
    rotation: 
      x: 0.193356517556
      y: 0.645613390532
      z: 0.235883157601
      w: 0.700111236194
    #
    # ]
"""
def tr2jac(T, samebody):
    # T = np.array(T)
    R = tr2r(T)
    # jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody == 1:
        jac_part1 = R.T
        jac_part2 = -numpy.dot(R.T, skew(transl(T)))
        jac_part3 = numpy.zeros((3, 3))
        jac_part4 = R.T

    else:
        jac_part1 = R.T
        jac_part2 = numpy.zeros((3, 3))
        jac_part3 = numpy.zeros((3, 3))
        jac_part4 = R.T
    jac_row1 = numpy.column_stack((jac_part1, jac_part2))
    jac_row2 = numpy.column_stack((jac_part3, jac_part4))
    jac = numpy.row_stack((jac_row1, jac_row2))
    return jac


def tr2jac_new(T, samebody):
    # T = np.array(T)
    R = tr2r(T)
    # jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody == 1:
        jac_part1 = R
        New_trans = numpy.dot(-1 * (R.I), transl(T))
        jac_part2 = -numpy.dot(R, skew(New_trans))
        # print "self.transl(T))",self.transl(T)
        # T1=[1,2,3]
        # print "self.skew(self.transl(T))\n",self.skew(New_trans)
        jac_part3 = numpy.zeros((3, 3))
        jac_part4 = R

    else:
        jac_part1 = R
        jac_part2 = numpy.zeros((3, 3))
        jac_part3 = numpy.zeros((3, 3))
        jac_part4 = R
    jac_row1 = numpy.column_stack((jac_part1, jac_part2))
    jac_row2 = numpy.column_stack((jac_part3, jac_part4))
    jac = numpy.row_stack((jac_row1, jac_row2))
    return jac


"""
if l is 3*1 , then get 
skew(l) = [ 0, -l(2), l(1)
            l(2), 0 , -l(0)
            -l(1), l(0), 0]
if l is 1*1, then get
skew(l) = [ 0 , -l[0]
            l[0], 0 ]

"""


def skew(l):
    a, b = numpy.shape(l)
    try:
        if a == 3:
            s = numpy.array([0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0])
            s = s.reshape((3, 3))
            # print "s:", s
            return s
        elif a == 1:
            s = numpy.array([0, -l[0], l[0], 0])
            s = s.reshape((2, 2))
            return s
    except:
        print("erro l size!!!  3*1 or 1*1 required!")


def tr2r( T):
    r = [0, 1, 2]
    c = [0, 1, 2]
    R1 = T[r]
    R = R1[:, c]
    return R


def transl(T):
    r = [3]
    c = [0, 1, 2]
    l1 = T[:, r]
    l = l1[c]
    return l