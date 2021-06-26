#! /usr/bin/python2.7

import numpy as np
import math
import sim as vrep
import time

import os
import ctypes
from pyquaternion import Quaternion as quat


jointName = ['joint_HL2','joint_HF2','joint_Knee2',\
        'joint_HL1','joint_HF1','joint_Knee1',\
            'joint_HL4','joint_HF4','joint_Knee4',\
                'joint_HL3','joint_HF3','joint_Knee3',]
direction = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]


class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]



def convert_type(input):
    ctypes_map = {int: ctypes.c_int,
                  float: ctypes.c_double,
                  str: ctypes.c_char_p
                  }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length == 0:
            print("convert type failed...input is "+input)
            return 0
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(
                    input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
            return arr
    else:
        if input_type in ctypes_map:
            return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
        else:
            print("convert type failed...input is "+input)
            return 0






if __name__ == '__main__':
    
    #for simulation
    freq = 500.0
    stand_kp = 100.0
    stand_kd = 1.0
    joint_kp = 0.0
    joint_kd = 0.05

    path = os.getcwd()
    #so_file = path + "/build/libihGaitCtrller.so"
    so_file = path + "/build/libironhorse_ctrl.so"
    print(os.getcwd())
    print(so_file)

    if(not os.path.exists(so_file)):
        print('cannot find cpp.so file')

    ih_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
    ih_gait_ctrller.toque_calculator.restype = ctypes.POINTER(StructPointer)

    print('cpp.so file is loaded')



