import math
from sympy import lambdify, var

from anomaly_common import theta, t, anomaly_h
from dataset import D
from regression import create_h
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

(time, atheta, xx, yy) = D

# fourier terms
M = 10
# polynomial degree
N = 1

h = create_h(M, N)  # Analytic Vector Function h
lh = [lambdify((theta, t), hi) for hi in h]
A = [[h1(th1, t1) for h1 in lh] for th1, t1 in zip(atheta, time)]
X = np.array(A, dtype=np.double)
# vector
qx1 = np.dot(X.T, xx)

print qx1
## Recursive method

# Vector q
k = len(time) - 50
X = np.array(A[:k], dtype=np.double)
qx2 = np.dot(X.T, xx[:k])


for i in range(k, len(time)):
    hk1 = np.array([hi(atheta[i], time[i]) for hi in lh], dtype=np.double)
    qx2 = qx2 + xx[i] * hk1


print qx2