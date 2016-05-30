import math
import numpy as np

import matplotlib.pylab as plt
from matplotlib import cm
from sympy import *
from sympy import init_printing

# pylab.rcParams['figure.figsize'] = (10.0, 10.0)

## nice printing in the notebook
init_printing()

#### Defining symbos t,and theta.
t = Symbol('t')
theta = Symbol('theta')
v2 = var('v^2')







########## anomaly ###########
def anomaly_t(fx, fy, t1, n=100, thet=None):
    """
    anomaly at time t1
    :param fx: x function
    :param fy: y function
    :param t1: time
    :param n: number of points for theta
    :return: anomaly polygon
    """
    if thet is None:
        thet = np.linspace(0, 2 * math.pi, n, endpoint=True)
        
    xx = [fx.subs({t: t1, theta: th1}).evalf() for th1 in thet]
    yy = [fy.subs({t: t1, theta: th1}).evalf() for th1 in thet]

    return xx, yy


def anomaly_h(pred_t, bx, by, hls, theta_lins):
    """
    Anmaly based on the regression.
    :param pred_t: time of the anomaly
    :param bx: beta constants for x
    :param by: beta constants for y
    :param hls: h functions
    :param theta_lins: linespace for theta.
    :return:
    """
    x2 = [np.sum([b1 * h1.subs({theta: th1, t: pred_t}).evalf() for b1, h1 in zip(bx, hls)])
          for th1 in theta_lins]
    y2 = [np.sum([b1 * h1.subs({theta: th1, t: pred_t}).evalf() for b1, h1 in zip(by, hls)])
          for th1 in theta_lins]

    return x2, y2


def anomaly_h_grid(grid, h, bx):
    TH, T = grid
    y_pred = np.array([[np.sum([b1 * h1.subs({theta: th1, t: t1}).evalf()
                                for b1, h1 in zip(bx, h)])
                        for t1, th1 in zip(t2, th2)]
                       for t2, th2 in zip(T, TH)]).astype('float')

    return y_pred


def robot_path(fx, fy, time, atheta):
    xx = [fx.subs({t: tt, theta: thetat}).evalf() for tt, thetat in zip(time, atheta)]
    yy = [fy.subs({t: tt, theta: thetat}).evalf() for tt, thetat in zip(time, atheta)]
    return xx, yy


def create_grid(time, thet=np.linspace(0, 2 * math.pi, 20)):
    TH, T = np.meshgrid(thet, time)
    return TH, T


def eval_fgrid(fx, grid):
    TH, T = grid
    Z = np.array([[fx.subs({t: t1, theta: th1}).evalf()
                   for t1, th1 in zip(t2, th2)]
                  for t2, th2 in zip(T, TH)]).astype('float')

    return Z

def create_space_h(h, TH, T):
    """
    Create space using the h function.
    It creates an array of two columns and n rows for samples.
    :param h:
    :param TH:
    :param T:
    :return:
    """
    th_t_space = np.vstack([TH.reshape(TH.size), T.reshape(T.size)]).T


    H_space = np.array([[ h1.subs({t:t1, theta: th1})
                      for h1 in h]
                    for th1, t1 in th_t_space]).astype('float')
    return H_space
