# -*- coding: utf-8 -*-            
# @Time : 2024/9/12 17:20
# @Author : wangfaqi
# @File ： leg.py

import math

import numpy as np


# 已知髋关节电机角度和连杆长度，求连杆其他物理状态
def getPhi(phi1, phi4, l1, l2, l3, l4):
    x_B = -0.06 + math.cos(phi1) * l1
    y_B = math.sin(phi1) * l1
    x_D = 0.06 + math.cos(phi4) * l4
    y_D = math.sin(phi4) * l4
    A_0 = 2 * l2 * (x_D - x_B)
    B_0 = 2 * l2 * (y_D - y_B)
    l_BD = ((x_D - x_B) ** 2 + (y_D - y_B) ** 2) ** 0.5
    C_0 = l2 ** 2 + l_BD ** 2 - l3 ** 2

    phi2 = 2 * math.atan2(B_0 + (A_0 ** 2 + B_0 ** 2 - C_0 ** 2) ** 0.5, A_0 + C_0)

    x_C = -0.06 + l1 * math.cos(phi1) + l2 * math.cos(phi2)
    y_C = 0 + l1 * math.sin(phi1) + l2 * math.sin(phi2)

    phi3 = math.atan2(y_C - y_D, x_C - x_D)

    l_0 = (x_C ** 2 + y_C ** 2) ** 0.5
    phi_0 = math.atan2(y_C, x_C)

    return phi2, phi3, l_0, phi_0


# Jacobi矩阵
def Mat_JRM(phi0, phi1, phi2, phi3, phi4, L0, l1, l4):
    JRM = np.matrix([[-l1 * math.sin(phi0 - phi3) * math.sin(phi1 - phi2) / math.sin(phi2 - phi3),
                      -l1 * math.sin(phi1 - phi2) * math.cos(phi0 - phi3) / (L0 * math.sin(phi2 - phi3))],
                     [-l4 * math.sin(phi0 - phi2) * math.sin(phi3 - phi4) / math.sin(phi2 - phi3),
                      -l4 * math.sin(phi3 - phi4) * math.cos(phi0 - phi2) / (L0 * math.sin(phi2 - phi3))]])
    return JRM


# 求解虚拟腿的腿长变化速度和腿摆角速度
def spd(dphi1, dphi4, l1, l2, l3, l4, l5, phi1, phi4):
    t8 = l2 ** 2
    t10 = l1 * math.cos(phi1)
    t11 = l4 * math.cos(phi4)
    t12 = l1 * math.sin(phi1)
    t13 = l4 * math.sin(phi4)
    t19 = l2 * t10 * 2.0
    t20 = l2 * t11 * 2.0
    t33 = t12 - t13
    t34 = l5 + t11 - t10
    t35 = t33 ** 2
    t36 = t34 ** 2
    t37 = l2 * t33 * 2.0
    t50 = t10 * t33 * 2.0 + t12 * t34 * 2.0
    t51 = t11 * t33 * 2.0 + t13 * t34 * 2.0
    t52 = t8 - l3 ** 2 + t35 + t36
    t54 = l2 * t12 * 2.0 + t50
    t55 = l2 * t13 * 2.0 + t51
    t60 = t50 * t52 * 2.0
    t61 = t51 * t52 * 2.0
    t58 = 1.0 / (l2 * t34 * 2.0 + t52)
    t59 = t58 ** 2
    t64 = t8 * t35 * 4.0 + t8 * t36 * 4.0 - t52 ** 2
    t69 = t8 * t10 * t33 * 8.0 + t8 * t12 * t34 * 8.0 - t60
    t70 = t8 * t11 * t33 * 8.0 + t8 * t13 * t34 * 8.0 - t61
    t65 = math.sqrt(t64)
    t90 = -t54 * t59 * (t37 - t65)
    t91 = -t55 * t59 * (t37 - t65)
    t72 = math.atan(-t58 * (t37 - t65))
    t94 = (1.0 / t65 * t69) / 2.0
    t95 = (1.0 / t65 * t70) / 2.0
    t80 = 1.0 / (t59 * (t37 - t65) ** 2 + 1.0)
    t78 = l2 * math.cos(t72 * 2.0)
    t79 = l2 * math.sin(t72 * 2.0)
    t86 = t12 + t79
    t88 = t10 - l5 / 2.0 + t78
    t112 = -t78 * t80 * (t90 + t58 * (t19 - t94))
    t113 = -t78 * t80 * (t91 + t58 * (t20 - t95))
    t114 = -t79 * t80 * (t90 + t58 * (t19 - t94))
    t115 = -t79 * t80 * (t91 + t58 * (t20 - t95))
    t100 = t12 + t79
    t103 = l5 / 2.0 - t10 - t78
    t104 = 1.0 / math.sqrt(t86 ** 2 + t88 ** 2)
    t105 = t103 ** 2
    t106 = 1.0 / t103
    t107 = 1.0 / t105
    t111 = 1.0 / (t100 ** 2 + t105)
    spd1 = (dphi1 * t104 * (
            t86 * (t10 - t78 * t80 * (t90 + t58 * (t19 - t94)) * 2.0) * 2.0 - t88 * (t12 - t79 * t80 * (t90 + t58 * (t19 - t94)) * 2.0) * 2.0)) / 2.0 + (
                   dphi4 * t104 * (t78 * t80 * t86 * (t91 + t58 * (t20 - t95)) * 4.0 - t79 * t80 * t88 * (t91 + t58 * (t20 - t95)) * 4.0)) / 2.0
    spd2 = dphi4 * t105 * t111 * (-t106 * t113 * 2.0 + t100 * t107 * t115 * 2.0) + dphi1 * t105 * t111 * (
            t106 * (t10 + t112 * 2.0) - t100 * t107 * (t12 + t114 * 2.0))
    return spd1, spd2
