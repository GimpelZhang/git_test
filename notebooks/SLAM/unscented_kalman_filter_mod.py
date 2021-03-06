"""

Unscented kalman filter (UKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

modification: Junchuan Zhang

"""

import math

import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg

# Covariance for UKF 协方差矩阵
# 运动过程协方差：
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
# 观测协方差
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter 仿真噪声参数
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2 # 输入噪声
GPS_NOISE = np.diag([0.5, 0.5]) ** 2 # GPS值噪声

DT = 0.1  # time tick [s]
SIM_TIME = 50  # simulation time [s]

#  UKF Parameter 无迹卡尔曼滤波参数设定值：
ALPHA = 0.001
BETA = 2
KAPPA = 0

show_animation = True


def calc_input():
    v = 1.0  # [m/s]
    yawRate = 0.1  # [rad/s]
    u = np.array([[v, yawRate]]).T
    return u


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2]), 0],
                  [DT * math.sin(x[2]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def generate_sigma_points(xEst, PEst, gamma):
    '''
    sigma点的生成：
    X_0 = 均值
    X_i = 均值 + (sqrt((nx+lamb)协方差))_i, for i=1,2,...,n
    X_i = 均值 - (sqrt((nx+lamb)协方差))_i-n, for i=n+1,n+2,...,2n
    
    返回的sigma矩阵形式举例：
    [[ 0.     0.002  0.     0.     0.    -0.002  0.     0.     0.   ]
     [ 0.     0.     0.002  0.     0.     0.    -0.002  0.     0.   ]
     [ 0.     0.     0.     0.002  0.     0.     0.    -0.002  0.   ]
     [ 0.     0.     0.     0.     0.002  0.     0.     0.    -0.002]]
    '''
    # X_0是均值
    sigma = xEst
    # 协方差矩阵的开平方：
    Psqrt = scipy.linalg.sqrtm(PEst)
    n = len(xEst[:, 0])
    # Positive direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst + gamma * Psqrt[:, i:i + 1]))
    # Negative direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst - gamma * Psqrt[:, i:i + 1]))
    return sigma


def predict_sigma_motion(sigma, u):
    """
        Sigma Points prediction with motion model
    """
    for i in range(sigma.shape[1]):
        sigma[:, i:i + 1] = motion_model(sigma[:, i:i + 1], u)

    return sigma


def predict_sigma_observation(sigma):
    """
        Sigma Points prediction with observation model
    """
    for i in range(sigma.shape[1]):
        sigma[0:2, i] = observation_model(sigma[:, i])

    sigma = sigma[0:2, :]

    return sigma


def calc_sigma_covariance(x, sigma, wc, Pi):
    nSigma = sigma.shape[1]
    d = sigma - x[0:sigma.shape[0]]
    P = Pi
    for i in range(nSigma):
        P = P + wc[0, i] * d[:, i:i + 1] @ d[:, i:i + 1].T
        
    return P


def calc_pxz(sigma, x, z_sigma, zb, wc):
    nSigma = sigma.shape[1]
    dx = sigma - x
    dz = z_sigma - zb[0:2]
    P = np.zeros((dx.shape[0], dz.shape[0]))

    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i:i + 1] @ dz[:, i:i + 1].T

    return P


def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma):
    '''
    z: 带噪声的观测
    u: 带噪声的输入
    wm, wc, gamma: 无迹变换参数
    '''
    #  Predict 预测
    # 生成sigma采样点： 2:
    sigma = generate_sigma_points(xEst, PEst, gamma)
    # 运动模型预测： 3:
    sigma = predict_sigma_motion(sigma, u)
    # 将采样点与权重相乘求对均值的估计： 4:
    xPred = (wm @ sigma.T).T
    # 求协方差的估计： 5: 
    PPred = calc_sigma_covariance(xPred, sigma, wc, Q)

    #  Update 更新
    # 对上一步计算过的均值的估计再次生成sigma采样点： 6:
    sigma = generate_sigma_points(xPred, PPred, gamma)
    # 用生成的sigma采样点做观测预测： 7:
    z_sigma = predict_sigma_observation(sigma)
    # 求均值：  8:
    zPred = (wm @ z_sigma.T).T
    # 求卡尔曼增益计算中的S：   9:
    st = calc_sigma_covariance(zPred, z_sigma, wc, R)
    # 求xz间协方差：   10: 
    Pxz = calc_pxz(sigma, xPred, z_sigma, zPred, wc)
    # 计算卡尔曼增益：  11:
    K = Pxz @ np.linalg.inv(st)
    y = z - zPred
    # 更新均值：  12: 
    xEst = xPred + K @ y
    # 更新协方差：  13: 
    PEst = PPred - K @ st @ K.T

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    rot = np.array([[math.cos(angle), math.sin(angle)],
                    [-math.sin(angle), math.cos(angle)]])
    fx = rot @ np.array([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def setup_ukf(nx):
    lamb = ALPHA ** 2 * (nx + KAPPA) - nx
    # calculate weights
    wm = [lamb / (lamb + nx)]
    wc = [(lamb / (lamb + nx)) + (1 - ALPHA ** 2 + BETA)]
    for i in range(2 * nx):
        wm.append(1.0 / (2 * (nx + lamb)))
        wc.append(1.0 / (2 * (nx + lamb)))
    gamma = math.sqrt(nx + lamb)

    wm = np.array([wm])
    wc = np.array([wc])

    return wm, wc, gamma


def main():
    print(__file__ + " start!!")

    nx = 4  # State Vector [x y yaw v]'
    xEst = np.zeros((nx, 1))
    xTrue = np.zeros((nx, 1))
    PEst = np.eye(nx)
    xDR = np.zeros((nx, 1))  # Dead reckoning
    # 初始化参数：
    wm, wc, gamma = setup_ukf(nx)

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    time = 0.0

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
