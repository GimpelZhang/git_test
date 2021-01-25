"""
Extended Kalman Filter SLAM example

author: Atsushi Sakai (@Atsushi_twi)

https://github.com/AtsushiSakai/PythonRobotics/tree/master/SLAM/EKFSLAM
"""

import math

import matplotlib.pyplot as plt
import numpy as np

# EKF state covariance: 3x3 size
Q = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2
R = np.diag([0.5, 0.5])**2

#  Simulation parameter
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
## 注：马哈拉诺比斯距离是由印度统计学家马哈拉诺比斯提出的，表示数据的协方差距离。它是一种有效的计算两个未知样本集的相似度的方法。与欧氏距离不同的是它考虑到各种特性之间的联系并且是尺度无关的，即独立于测量尺度（来自wiki）。 
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

show_animation = True


def ekf_slam(xEst, PEst, u, z):
    # Predict
    # 预测过程只与运动模型有关
    S = STATE_SIZE
    # 返回运动模型雅克比G和对角矩阵Fx
    G, Fx = jacob_motion(xEst[0:S], u)
    # 返回机器人位姿的在含噪控制下的预测：
    xEst[0:S] = motion_model(xEst[0:S], u)
    # 更新机器人位姿的P
    PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Q @ Fx
    # 给Landmark准备的variance
    initP = np.eye(2) 

    # Update
    # 对在此时刻新得到的一组z含噪观测，对z中的每一个元素做操作：
    for iz in range(len(z[:, 0])):  # for each observation
        min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_lm(xEst)
        # 在函数search_correspond_landmark_id中，把M_DIST_TH放在了最后一个，又因为已有的LM的序号值范围是（0至（nLM-1）），所以最后一个值M_DIST_TH的序号就是nLM，所以如果min_id等于nLM，意味着新观测z[iz, 0:2]与所有已有观测的马哈拉诺比斯距离都超过了M_DIST_TH，即这是一个以前没看到过的LM：
        if min_id == nLM:
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        # 如果min_id不等于nLM，则认为这是一个已经观测过的LM，马哈拉诺比斯距离最小的那个，也就是min_id指向的那个就是对应的已观测LM：
        lm = get_landmark_position_from_state(xEst, min_id)
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)

        K = (PEst @ H.T) @ np.linalg.inv(S)
        xEst = xEst + (K @ y)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):
    """
    仿真过程
    """
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
            angle_n = angle + np.random.randn() * Q_sim[1, 1] ** 0.5  # add noise
            zi = np.array([dn, angle_n, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5,
        u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5]]).T

    xd = motion_model(xd, ud)
    # 返回xTrue(3x1), z含噪观测（3xnLM含RFID的id）,xd(3x1),ud含噪控制(2x1)
    return xTrue, z, xd, ud


def motion_model(x, u):
    """
    根据控制预测机器人位姿
    """
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    # 返回3x1
    return x


def calc_n_lm(x):
    """
    返回Landmark的个数
    """
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=np.float64)

    G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx
    # 返回雅克比矩阵G(3x3)，和对角矩阵Fx(实际上的np.eye(3))
    return G, Fx,


def calc_landmark_position(x, z):
    """
    根据观测z和预测后的x状态向量，计算LM的坐标位置
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp


def get_landmark_position_from_state(x, ind):
    """
    根据lm在状态向量中的id查找lm的坐标：
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_landmark_id(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """
    # 此时已有的LM的个数：
    nLM = calc_n_lm(xAug)

    min_dist = []

    for i in range(nLM):
        # 返回第i个已有LM的坐标lm：
        lm = get_landmark_position_from_state(xAug, i)
        # 放入第i个已有LM的坐标lm、机器人位姿预测过的状态向量xAug和variance PAug、待比较的新观测zi
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id


def calc_innovation(lm, xEst, PEst, z, LMid):
    """
    Calculates the innovation based on expected position and landmark position
    
    :param lm:   landmark position
    :param xEst: estimated position/state
    :param PEst: estimated covariance
    :param z:    read measurements
    :param LMid: landmark id
    :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
    """
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
    # 放入delta的平方（标量）,delta(2x1),状态向量和lm在状态向量中的序号+1
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + R

    return y, S, H


def jacob_h(q, delta, x, i):
    """
    Calculates the jacobian of the measurement function
    
    :param q:     the range from the system pose to the landmark
    :param delta: the difference between a landmark position and the estimated system position
    :param x:     the state, including the estimated system position
    :param i:     landmark id + 1
    :returns:     the jacobian H
    """
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])
    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))

    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
    F = np.vstack((F1, F2))
    H = G @ F

    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input() #真值

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)
        # 返回xTrue(3x1), z含噪观测（3xnLM含RFID的id）,xDR(3x1),ud含噪控制(2x1)

        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            # plot landmark
            for i in range(calc_n_lm(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b")
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k")
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
