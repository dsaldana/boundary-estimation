from sympy import *
from sympy import init_printing
import sympy as sp
import math

import pylab
import numpy as np
import matplotlib.pylab as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D


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

########### plotting ########

def draw_anomaly(fx, fy, t1, n=100, line='.-'):
    xx, yy = anomaly_t(fx, fy, t1, n)
    plt.plot(xx, yy, line, label='$t=%d$'%t1)
    plt.xlabel('$x$')
    plt.ylabel('$y$')


def draw_path(fx, fy, time, atheta):
    xx, yy = robot_path(fx, fy, time, atheta)
    plt.plot(xx, yy, '.-')

    plt.xlabel('$x$')
    plt.ylabel('$y$')


# def draw_anomaly_3d


def draw3d_cosx():
    ######## Doing
    #### theta = cos(theta) and Z = t*theta
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    thet = np.cos(np.linspace(0, 2 * math.pi, 20))
    TH, T = np.meshgrid(thet, time)

    ## plot surface for X function
    Z = T * (TH)
    surf = ax.plot_surface(TH, T, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                           linewidth=0, antialiased=False)

    # plot robot path
    ax.plot3D(np.cos(atheta), time, time * np.cos(atheta), 'o', label='robot path')
    plt.legend()

    # Labels
    plt.xlabel('$\\theta$', fontsize=14, color='blue')
    plt.ylabel('time', fontsize=14, color='red')
    fig.colorbar(surf, shrink=0.5, aspect=5)


def draw3d_surface(grid, Z, xlabel='$s$', ylabel='$t$', zlabel='$x$'):
    # %pylab tk
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    TH, T = grid

    ## plot surface for X function
    surf = ax.plot_surface(TH, T, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                           linewidth=0, antialiased=False)


    # Labels
    plt.xlabel(xlabel, fontsize=20)
    plt.ylabel(ylabel, fontsize=20)
    ax.set_zlabel(zlabel,fontsize=20)
    
    fig.colorbar(surf, shrink=0.5, aspect=5)

    return fig


def draw3d_path(atheta, time, xx, fig=plt.figure()):
    ax = fig.gca(projection='3d')
    ax.plot3D(atheta, time, xx, 'o', label='robot path')
    plt.legend()



## for debug only ##
def approx_f_with_sins():
    """
    Approximates a function with sin and cos

    """
    btheta = np.linspace(0,.5*math.pi,100)

    g = btheta
    #gy = np.sin(g)*np.cos(5*g)
    gy = np.cos(g)**(2./9)
    plt.plot(g, gy)

    h = [1 + 0*theta,
        sin(theta),
        cos(theta),
        sin(2*theta),
        cos(2*theta),
        sin(3*theta),
        cos(3*theta),
        sin(4*theta),
        cos(4*theta),
        sin(5*theta),
        cos(5*theta),
        sin(6*theta),
        cos(6*theta),
        sin(7*theta),
        cos(7*theta),
        sin(8*theta),
        cos(8*theta),
        sin(9*theta),
        cos(9*theta),
        ]



    A = [[h1.subs({theta:th1}).evalf() for h1 in h]
             for th1 in btheta]


    Bx = np.array(gy).T
    bx, residualsx, rankx, singularsx = np.linalg.lstsq(A,Bx)
    #plt.plot(bx)
    x2 = [np.sum([b1 * h1.subs({theta:th1}).evalf() for b1, h1 in zip(bx, h)])
              for th1 in g]

    plt.plot(g, x2, 'o-')
    bx

