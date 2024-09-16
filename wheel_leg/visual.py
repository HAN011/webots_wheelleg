
import matplotlib.pyplot as plt

# 初始化数据列表
set_data, y1_data, y2_data, y3_data, y4_data, y5_data, y6_data,y7_data,y8_data,y9_data,y10_data,y11_data,y12_data = [], [], [], [], [], [], [], [], [], [], [], [], []
data_y={
    0:y1_data,
    1:y6_data,
    2:y2_data,
    3:y7_data,
    4:y3_data,
    5:y8_data,
    6:y4_data,
    7:y9_data,
    8:y5_data,
    9:y10_data,
    10:y1_data,
    11:y1_data,
    12:y1_data,
    13:y1_data,
    14:y1_data,
    15:y1_data
}

fig, axs = plt.subplots(2, 3, figsize=(10, 8))

# 创建线条对象
lines = [
    axs[0, 0].plot([], [], color='r')[0],  # T1 Left - 红色
    axs[0, 0].plot([], [], color='k')[0],  # T1 Left - 红色

    axs[0, 1].plot([], [], color='r')[0],  # T1 Right - 绿色
    axs[0, 1].plot([], [], color='k')[0],  # T1 Right - 绿色

    axs[1, 0].plot([], [], color='g')[0],  # T2 Left - 蓝色
    axs[1, 0].plot([], [], color='k')[0],  # T1 Right - 绿色

    axs[1, 1].plot([], [], color='g')[0],  # T2 Right - 黄色
    axs[1, 1].plot([], [], color='k')[0],  # T1 Right - 绿色

    axs[0, 2].plot([], [], color='c')[0],  # Wheel Left - 洋红色
    axs[0, 2].plot([], [], color='k')[0],  # T1 Right - 绿色

    axs[1, 2].plot([], [], color='c')[0],   # Wheel Right - 青色
    axs[1, 2].plot([], [], color='k')[0],  # T1 Right - 绿色
]

def visualization(time,state,ref):
    axs[0, 0].set_title('s')
    axs[0, 1].set_title('phi')
    axs[0, 2].set_title('theta_ll')
    axs[1, 0].set_title('theta_lr')
    axs[1, 1].set_title('theta_b')

    set_data.append(time)
    #系统实际量
    y1_data.append(state[0])
    y2_data.append(state[2])
    y3_data.append(state[4])
    y4_data.append(state[6])
    y5_data.append(state[8])
    #设定量 ref
    y10_data.append(ref[8])
    y6_data.append(ref[0])
    y7_data.append(ref[2])
    y8_data.append(ref[4])
    y9_data.append(ref[6])

    if len(set_data) > 100:
        for i, data in enumerate([set_data] + [y1_data, y2_data, y3_data, y4_data, y5_data, y6_data,y7_data, y8_data, y9_data, y10_data]):
            #if data:
                data.pop(0)

    #更新数据
    for i in range(6):
        lines[2*i].set_data(set_data, data_y[i*2])
        lines[2*i+1].set_data(set_data, data_y[i*2+1])

    #重新绘制子图
    for ax, line in zip(axs.flatten(), lines):
        ax.relim()
        ax.autoscale_view()


    plt.draw()
    plt.pause(0.001)
    return 0

