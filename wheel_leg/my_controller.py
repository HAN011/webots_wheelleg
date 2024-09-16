# -*- coding: utf-8 -*-            
# @Time : 2024/9/12 17:21
# @Author : wangfaqi
# @File ： my_controller.py

import math
## 可视化使用
import matplotlib.pyplot as plt
import time
##
import sys
import lagrange
import leg
import mymath
from controller import Motor, PositionSensor, Gyro, Accelerometer, Robot
import visual
import numpy as np  # 导入 NumPy

robot = Robot()
timestep = int(robot.getBasicTimeStep())

list_keyboard ={'a':0,'s':0,'d':0,'q':0,'e':0,'w':0,'z':0,'x':0,'c':0,'r':0,'f':0,'v':0}
items = list_keyboard.items()
import threading
import time
import keyboard
def keyboard_input_thread(robot):
    while True:
        def on_key_event(e):
            print(f"Key {e.name} pressed")
            for key,value in items:
                list_keyboard[key]+=keyboard.is_pressed(key)
            print(cmd_phi)
        keyboard.on_press(on_key_event)
        keyboard.wait('esc')  # 监听 'esc' 键退出
        # 添加其他命令处理
        time.sleep(0.1)  # 稍微延迟，避免CPU占用过高

# 创建并启动键盘输入线程
input_thread = threading.Thread(target=keyboard_input_thread, args=(robot,))
input_thread.start()

# 输出
i=0
j=0

# 足电机
motor_5 = Motor('motor5')   #左轮
motor_6 = Motor('motor6')   #右轮

# 髋电机
motor_1 = Motor('motor1')
motor_3 = Motor('motor3')
motor_2 = Motor('motor2')
motor_4 = Motor('motor4')

# 足角度
ps_5 = PositionSensor('ps5')
ps_6 = PositionSensor('ps6')

# 髋角度
ps_1 = PositionSensor('ps1')
ps_3 = PositionSensor('ps3')
ps_2 = PositionSensor('ps2')
ps_4 = PositionSensor('ps4')

gyro = Gyro('gyro')

accelerometer = Accelerometer('accelerometer')

# 初始化电机
for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0)

# 初始化传感器
for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, accelerometer]:
    sensor.enable(timestep)

# 差分初始化
d_t = timestep / 1000
Theta_b = mymath.Discreteness(d_t)

Fai = mymath.Discreteness(d_t)
Theta_wl = mymath.Discreteness(d_t)
Theta_wr = mymath.Discreteness(d_t)
Roll = mymath.Discreteness(d_t)

theta_l1 = mymath.Discreteness(d_t)
theta_l4 = mymath.Discreteness(d_t)
theta_r1 = mymath.Discreteness(d_t)
theta_r4 = mymath.Discreteness(d_t)

d_Ll = mymath.Discreteness(d_t)
d_Lr = mymath.Discreteness(d_t)

Ll = mymath.Discreteness(d_t)

Ll.last_diff = 0.10456422824432195

theta_l1.last_diff = 3.447025
theta_r1.last_diff = 3.447025
theta_l4.last_diff = -0.305426
theta_r4.last_diff = -0.305426

target_L0_l = 0.2
target_L0_r = 0.2
target_roll = 0.0

# PID初始化
F0_control_l = mymath.PID_control(400, 2, 8000, target_L0_l)
F0_control_r = mymath.PID_control(400, 2, 8000, target_L0_r)

Froll_control = mymath.PID_control(3000, 1, 100, target_roll)

r = 0.10
current_time = 0

# 离地标志
Offground_l = 0
Offground_r = 0

## for kalman
x_k = np.matrix([[0], [0]])
x_hat_k = np.matrix([[0], [0]])

Q_c = np.matrix([[1 / 4 * d_t ** 4, d_t ** 3 / 2], [d_t ** 3 / 2, d_t ** 2]])
R_c = np.matrix([[1, 0], [0, 1]])
P = np.matrix([[1, 0], [0, 1]])
H_m = np.matrix([[1, 0], [0, 1]])

A = np.matrix([[1, d_t], [0, 1]])
B = np.matrix([[0], [0]])

# Main loop:
while robot.step(timestep) != -1:
    cmd_s=(list_keyboard['w']-list_keyboard['s'])/10.0
    cmd_phi=(list_keyboard['a']-list_keyboard['d'])/5.0
    current_time = current_time + d_t

    # 机器人基本状态获取

    dot_theta_b = gyro.getValues()[0]
    theta_b = Theta_b.Sum(dot_theta_b)

    dot_roll = gyro.getValues()[1]
    roll = Roll.Sum(dot_roll)

    dot_fai = gyro.getValues()[2]
    fai = Fai.Sum(dot_fai)

    acc_x = accelerometer.getValues()[0]
    acc_y = -accelerometer.getValues()[1]
    acc_z = accelerometer.getValues()[2]

    # print("加速度",acc_x,acc_y,acc_z)

    phi_l1 = 3.447025 - ps_1.getValue()
    phi_l4 = -0.305426 - ps_3.getValue()

    phi_r1 = 3.447025 - ps_2.getValue()
    phi_r4 = -0.305426 - ps_4.getValue()

    phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.2, 0.3, 0.3, 0.2)
    phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.2, 0.3, 0.3, 0.2)

    theta_ll = 1.570796 - phi0_l + theta_b
    theta_lr = 1.570796 - phi0_r + theta_b

    theta_wl = ps_5.getValue()
    theta_wr = ps_6.getValue()

    dot_theta_wl = Theta_wl.Diff(theta_wl)
    dot_theta_wr = Theta_wr.Diff(theta_wr)

    omega1l = theta_l1.Diff(phi_l1)
    omega4l = theta_l4.Diff(phi_l4)
    omega1r = theta_r1.Diff(phi_r1)
    omega4r = theta_r4.Diff(phi_r4)

    L0_speedl, test2 = leg.spd(omega1l, omega4l, 0.2, 0.3, 0.3, 0.2, 0.12, phi_l1, phi_l4)
    L0_speedr, test4 = leg.spd(omega1r, omega4r, 0.2, 0.3, 0.3, 0.2, 0.12, phi_r1, phi_r4)

    L0_spdl = Ll.Diff(L0_l)

    L0_accl = d_Ll.Diff(L0_speedl)
    L0_accr = d_Lr.Diff(L0_speedr)

    dot_theta_ll = test2 + dot_theta_b
    dot_theta_lr = test4 + dot_theta_b

    # print("角度，", phi_l1, phi_l4, phi_r1, phi_r4)
    # print("角速度，", omega1l, omega4l, omega1r, omega4r)

    # 状态方程的十个状态量

    s = r * (theta_wl + theta_wr) / 2
    dot_s_b = r * (dot_theta_wl + dot_theta_wr) / 2

    dot_s = dot_s_b + 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll) + L0_r * dot_theta_lr * math.cos(theta_lr)) + 0.5 * (L0_speedl * math.sin(theta_ll) + L0_speedr * math.sin(theta_lr))

    x_k = np.matrix([[dot_s], [acc_y]])
    x_k = A * x_k
    z = H_m * x_k

    x_hat_k, x_hat_minus, P = mymath.LinearKalman(A, B, Q_c, R_c, H_m, z, x_hat_k, P)
    dot_s_k = x_hat_k[0, 0]
    # print(dot_s)


    real_state = np.matrix([[s],
                            [dot_s_k],
                            [fai],
                            [dot_fai],
                            [theta_ll],
                            [dot_theta_ll],
                            [theta_lr],
                            [dot_theta_lr],
                            [theta_b],
                            [dot_theta_b]])

    # print("状态，", s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll, theta_lr, dot_theta_lr, theta_b, dot_theta_b)
    # print(L0_l,L0_r)
    pass_key = 0

    # 没有遥控，所以根据时间规定状态（开环）
    if 0 < current_time <= 1:
        expect_state = np.matrix([[0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    elif 1 < current_time <= 6:
        expect_state = np.matrix([[1 * (current_time - 1)],
                                  [1],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    elif 6 < current_time <= 8:
        expect_state = np.matrix([[1 * (current_time - 1)],
                                  [1],
                                  [-1.57 * (current_time - 6)],
                                  [-1.57],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    elif 8 < current_time <= 11:
        expect_state = np.matrix([[1 * (current_time - 1)],
                                  [1],
                                  [-3.14],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    elif 11 < current_time <= 48:
        expect_state = np.matrix([[10+cmd_s],
                                  [0],
                                  [-3.14+cmd_phi],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    else:
        break

    if 12 < current_time <= 14:
        F0_control_l.targ_value = 0.205
        F0_control_r.targ_value = 0.205
    if 14 < current_time <= 16:
        F0_control_l.targ_value = 0.325
        F0_control_r.targ_value = 0.325
    if 16 < current_time <= 48:
        F0_control_l.targ_value = 0.205
        F0_control_r.targ_value = 0.203

    K = lagrange.K(target_L0_l, target_L0_r)

    if Offground_l:
        K[0] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        K[2] = [0, 0, 0, 0, K[2, 4], K[2, 5], 0, 0, 0, 0]

    if Offground_r:
        K[1] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        K[3] = [0, 0, 0, 0, 0, 0, K[3, 6], K[3, 7], 0, 0]

    # 解算力矩
    U = K * (expect_state - real_state)

    T_l = float(U[0][0])
    T_r = float(U[1][0])
    T_pl = float(U[2][0])
    T_pr = float(U[3][0])

    JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.2, 0.2)
    JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.2, 0.2)

    dF_0_l = -F0_control_l.position_pid(L0_l)
    dF_0_r = -F0_control_r.position_pid(L0_r)

    # print("PID_df,", dF_0_l, dF_0_r)

    dF_roll = -Froll_control.position_pid(roll)
    # dF_roll = 0

    # R_l = 0.2265

    F_blinl = 0
    F_blinr = 0

    # F_bl = -dF_roll + dF_0_l - 6 * 9.8 / math.cos(theta_ll) + F_blinl
    # F_br = dF_roll + dF_0_r - 6 * 9.8 / math.cos(theta_lr)  - F_blinr

    F_bl = -dF_roll + dF_0_l - 30 + F_blinl
    F_br = dF_roll + dF_0_r - 30 - F_blinr

    # print(F_bl,F_br,T_pr,T_pl)

    #这里需要注意力矩的正负号问题
    T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
    T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

    T1_l = float(T_JOINT_l[0][0])
    T1_r = float(T_JOINT_l[1][0])

    T2_l = float(T_JOINT_R[0][0])
    T2_r = float(T_JOINT_R[1][0])

    if i<50 :
        i+=1
    else:
        #print("力矩， {0:10.2f} {1:10.2f} {2:10.2f} {3:10.2f} {4:10.2f} {5:10.2f}".format(T1_l, T1_r, T2_l, T2_r, T_l,T_r) + "{:37.2f}".format(current_time))
        i=0

    # (acc_z - 9.8) 需要结合机身的姿态矩阵得出竖直向下的加速度，这里简单处理了

    acc_zwl = (acc_z - 9.8) - L0_accl * math.cos(theta_ll)
    acc_zwr = (acc_z - 9.8) - L0_accr * math.cos(theta_lr)

    Pl = F_bl * math.cos(theta_ll) + T_pl * math.sin(theta_ll) / L0_l
    Pr = F_br * math.cos(theta_lr) + T_pr * math.sin(theta_lr) / L0_r

    m_w = 1

    F_nl = Pl - m_w * 9.8 - m_w * acc_zwl
    F_nr = Pr - m_w * 9.8 - m_w * acc_zwr

    # print("离地有关，", L0_l, L0_r, acc_zwl, acc_zwr, F_bl, F_br, T_pl, T_pr, Pl, Pr, F_nl, F_nr)
    # data = [L0_l, L0_r, acc_zwl, acc_zwr, F_bl, F_br, T_pl, T_pr, Pl, Pr, F_nl, F_nr]
    #
    # # with open('data.csv', 'a', newline = '') as file:
    # #     writer = csv.writer(file)
    # #     writer.writerow(data)
    if j <5 :
        j+=1
        # print (j)
    else :
        # visual.visualization(current_time,real_state,expect_state)
        j=0

    #
    # x_data.append(time.time())
    # y_data.append(T1_l)
    #
    # plt.plot(x_data, y_data)
    # plt.draw()
    # plt.pause(0.01)

    if F_nl > -7:
        Offground_l = 1
    else:
        Offground_l = 0

    if F_nr > -7:
        Offground_r = 1
    else:
        Offground_r = 0

    # 最终赋值（包含开环跳跃）
    if 0 < current_time <= 10 * d_t:
        motor_5.setTorque(T_l)
        motor_6.setTorque(T_r)
    elif 5.10 < current_time <= 5.20:
        motor_1.setTorque(15)
        motor_3.setTorque(-15)
        motor_2.setTorque(15)
        motor_4.setTorque(-15)
        motor_5.setTorque(T_l)
        motor_6.setTorque(T_r)
    elif 5.20 < current_time <= 5.30:
        motor_1.setTorque(-10)
        motor_3.setTorque(10)
        motor_2.setTorque(-10)
        motor_4.setTorque(10)
        motor_5.setTorque(T_l)
        motor_6.setTorque(T_r)
    else:
        motor_1.setTorque(T1_l)
        motor_3.setTorque(T1_r)
        motor_2.setTorque(T2_l)
        motor_4.setTorque(T2_r)
        motor_5.setTorque(T_l) #左轮
        motor_6.setTorque(T_r) #右轮

    pass


sys.exit()