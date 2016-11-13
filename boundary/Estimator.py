import math

from sympy import lambdify, var, diff, Matrix
from sympy.tensor.array import Array
from sympy.vector import Vector

from boundary.util.anomaly_common import theta, t, anomaly_h
from dataset import D
from boundary.util.regression import create_h
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt


class Estimator(object):
    def __init__(self, N, M):
        self.M = M  # Parameters of the fourier series
        self.N = N  # Parameters of the polynomial
        self.h = create_h(M, N)  # Analytic Vector Function h
        self.lh = lambdify((t, theta), [hi for hi in self.h], modules='numpy')  # Lambdified h
        # derivative on time
        # self.dh = diff(self.h, t)
        # self.ldh = lambdify((theta, t), self.dh.T.tolist()[0], modules='numpy')

        # Variables for estimation
        self.P = None  # P matrix (with inverse)
        self.qx, self.qy = None, None  # q vector for x and for y
        self.bx, self.by = None, None  # Vector of weights
        self.std = None  # Standard deviation

    def initial_estimation(self, t_path, s_path, xx, yy):
        """
        Uses the initial dataset composed by a location x,y, its parameter s, and the time
        :param t_path: (array) time of the sample
        :param s_path: (array)  parameter of the curve
        :param xx: (array) x coordinates
        :param yy: (array) y coordinates
        """
        # Design matrix
        A = np.vstack([self.lh(t1, th1) for th1, t1 in zip(s_path, t_path)])

        X = np.array(A)
        # Recursive matrix
        self.P = inv(np.dot(X.T, X))
        # Recursive vector
        self.qx = np.dot(X.T, xx)
        self.qy = np.dot(X.T, yy)

        # Standard deviation
        estimation_x, estimation_y = self.get_estimation(t_path, s_path)  # Estimator \mu
        ex = np.array(estimation_x) - np.array(xx)  # error x
        ey = np.array(estimation_y) - np.array(yy)  # error y
        e = np.sqrt(sum(ex ** 2) + sum(ey ** 2))

        # cycle_lin_s, e = self.get_bias(t_path, s_path, xx, yy)
        self.std = math.sqrt(1. / len(t_path)) * (e)

    def initial_estimation2(self, t_path, s_path, zx, zy, dzx=None, dzy=None):
        """
        Uses the initial dataset composed by a location x,y, its parameter s, and the time.
        It can also includes information about the movement of the point. the velocity is dzx, dzy.

        :param t_path: (array) time of the sample
        :param s_path: (array)  parameter of the curve
        :param zx: (array) x coordinates
        :param zy: (array) y coordinates
        :param dzx: (array) velocity of zx.
        :param dzy: (array) velocity of zy
        """
        # Design matrix
        A = [[h1(t1, th1) for h1 in self.lh] for th1, t1 in zip(s_path, t_path)]

        X = np.array(A)
        # Recursive matrix
        self.P = inv(np.dot(X.T, X))
        # Recursive vector
        self.qx = np.dot(X.T, zx)
        self.qy = np.dot(X.T, zy)

        # Standard deviation
        estimation_x, estimation_y = self.get_estimation(t_path, s_path)  # Estimator \mu
        ex = np.array(estimation_x) - np.array(zx)  # error x
        ey = np.array(estimation_y) - np.array(zy)  # error y
        e = np.sqrt(sum(ex ** 2) + sum(ey ** 2))

        # cycle_lin_s, e = self.get_bias(t_path, s_path, xx, yy)
        self.std = math.sqrt(1. / len(t_path)) * (e)

    def get_estimation_t(self, fixed_t, thetas):
        """
        Get an estimation for a fixed time and different thetas
        :param fixed_t:
        :param thetas:
        :return:
        """
        return self.get_estimation(fixed_t * np.ones(len(thetas)), thetas)

    def get_estimation(self, times, lin_s):
        """
        Get an estimation for different times and different thetas (like a path).
        :param times:
        :param lin_s:
        :return:
        """
        self.bx = np.dot(self.P, self.qx)
        self.by = np.dot(self.P, self.qy)

        ## anomaly based on the least squares parameters
        x2 = [np.sum([bi * hi for bi, hi in zip(self.bx, self.lh(ti, th1))])
              for ti, th1 in zip(times, lin_s)]
        # x21 = [np.dot(self.bx, self.lh(ti, th1)) for ti, th1 in zip(times, lin_s)]
        y2 = [np.sum([bi * hi for bi, hi in zip(self.by, self.lh(ti, th1))])
              for ti, th1 in zip(times, lin_s)]

        return x2, y2

    def get_tanget_t(self, fixed_t, thetas):
        """
        Get an estimation for a fixed time and different thetas
        TODO: it was not tested properly
        :param fixed_t:
        :param thetas:
        :return:
        """
        return self.get_tangent(fixed_t * np.ones(len(thetas)), thetas)

    def get_tangent(self, times, lin_s):
        """
        Get an estimation for different times and different thetas (like a path).
        TODO: it was not tested properly
        :param times:
        :param lin_s:
        :return:
        """
        self.bx = np.dot(self.P, self.qx)
        self.by = np.dot(self.P, self.qy)

        dh = diff(Matrix(self.h), theta)
        ldh = [lambdify((theta, t), hi) for hi in dh]

        ## anomaly based on the least squares parameters
        x2 = [np.sum([bi * hi(th1, ti) for bi, hi in zip(self.bx, ldh)])
              for ti, th1 in zip(times, lin_s)]
        y2 = [np.sum([bi * hi(th1, ti) for bi, hi in zip(self.by, ldh)])
              for ti, th1 in zip(times, lin_s)]

        return x2, y2

    def get_bias(self, times, lin_s, xx, yy):
        """
        Compute the bias based on the last cycle of the dataset.
        :param times:
        :param lin_s:
        :param xx:
        :param yy:
        """

        # Compute the last cycle.
        last_theta = lin_s[-1]
        # Lin space for the las cycle
        cycle_lin_s = lin_s[lin_s > last_theta - 2 * math.pi]
        cnp = len(cycle_lin_s)  # Cycle number of points
        ctimes = times[-cnp:]

        # samples points in the last cycle
        points_xx = xx[-cnp:]
        points_yy = yy[-cnp:]
        # Estimation in the last cycle of the path
        estimation_x, estimation_y = self.get_estimation(ctimes, cycle_lin_s)

        # Error
        ex = np.array(estimation_x) - np.array(points_xx)
        ey = np.array(estimation_y) - np.array(points_yy)

        e = np.sqrt(ex ** 2 + ey ** 2)

        return cycle_lin_s, e

    def get_variance(self, t0, lin_s):
        """
        Variance for the estimation. Given a time value, it returns
        the variance at every point s of lins.
        :param t0: time
        :param lin_s: lin space for s.
        """
        std2 = self.std ** 2  # TODO Compute iterative variance

        epx = []

        for si in lin_s:
            xe = np.array([hi.subs(t, t0).subs(theta, si) for hi in self.h], dtype=np.float32)
            epxi = std2 * np.sqrt(np.dot(np.dot(xe.T, self.P), xe))
            epx.append(epxi)
        return epx

    def add_sample_point(self, t0, s0, x0, y0):
        """
        Iterative least squares
        :param t0:
        :param s0:
        :param x0:
        :param y0:
        """
        hk1 = np.array([hi(s0, t0) for hi in self.lh])

        # Matrix P
        Ph = np.dot(self.P, hk1)
        Ph.shape = (len(Ph), 1)
        self.P = self.P - np.dot(Ph, Ph.T) / (1 + np.dot(hk1, Ph))

        # Vector q
        self.qx = self.qx + x0 * hk1
        self.qy = self.qy + y0 * hk1

        pass


# fourier terms
M = 10
# polynomial degree
N = 1
(time, atheta, xx, yy) = D

(time, atheta, xx, yy) = (time[:], atheta[:], xx[:], yy[:])

estimator = Estimator(N, M)

estimator.initial_estimation(time, atheta, xx, yy)
# print estimator.std

x1, y1 = estimator.get_estimation_t(100, np.linspace(0, 2 * math.pi, 100))
plt.plot(x1, y1)

# plt.show()

# ## Test iterative
# estimator2 = Estimator(N, M)
# k = len(time) - 50
#
# estimator2.initial_estimation(time[:k], atheta[:k], xx[:k], yy[:k])

# for i in range(k, len(time)):
#     estimator2.add_sample_point(time[i], atheta[i], xx[i], yy[i])
#
#     lin_s0 = np.linspace(0, 2 * math.pi, 100)
# #
# #     plt.plot(lin_s0, var, '-',label=str(i))
# #     var = estimator2.get_variance(time[i], lin_s0)
# #     plt.plot(lin_s0, var, '-',label=str(i))
# # #
# #     plt.legend()
# #     plt.show()
#
#     x1, y1 = estimator2.get_estimation_t(100, np.linspace(0, 2 * math.pi, 100))
# # plt.plot(np.linspace(0, 2 * math.pi, 100), x1)
#     plt.plot(x1, y1)
#     plt.show()
#
#
# x1, y1 = estimator.get_estimation_t(100, np.linspace(0, 2 * math.pi, 100))
# plt.plot(x1, y1)
# # plt.plot(np.linspace(0, 2 * math.pi, 100), x1)
# # plt.show()
# #
# #
# # print estimator.qx[0]
# # print estimator2.qx[0]
# # #estimator2.P = estimator.P
# x1, y1 = estimator2.get_estimation_t(100, np.linspace(0, 2 * math.pi, 100))
# # plt.plot(np.linspace(0, 2 * math.pi, 100), x1)
# plt.plot(x1, y1)
# plt.show()

# the, e = estimator.get_bias(time, atheta, xx, yy)
# plt.plot(the, e)
# plt.show()


lin_s0 = np.linspace(0, 2 * math.pi, 100)
var_s = estimator.get_variance(time[-1], lin_s0)

# plt.plot(lin_s0, var, 'o')
# plt.show()



# plt.plot(x1, y1,  label='$\gamma$')  # Real
# plt.plot(x1, y1, 'y', linewidth=2, label='$\hat\gamma$')  #Estimation
#
# plt.legend()
#
# # Uncertainty
# fig = plt.gcf()

# for xi, yi, epi in zip(x1, y1, var_s):
#     circle1=plt.Circle((xi,yi), 2*math.sqrt(epi), color='g')
#     fig.gca().add_artist(circle1)

# Last robot location
# plt.plot(xx[-1], yy[-1], 'rv')
# plt.show()
