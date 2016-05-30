from anomaly_common import *


def create_h(m, n):
    """
    Create the h function for estimating the boundary behavior.
    m: number of sins
    n: polinomial degree
    @return an array with (2m+1)(n+1) terms.
    """
    # 2m+1 terms
    h_fourier = [1] + [sin(j * theta) for j in range(1, m + 1)] + [cos(j * theta) for j in range(1, m + 1)]

    # n+1 terms
    h_poly = [t ** i for i in range(n + 1)]

    h = [f * g for g in h_fourier for f in h_poly]

    return h


def create_matrix_A(h_functions, atheta, time):
    """
    Create a matrix A with rows equal to the number of rows in atheta/time.
    the number of colums are the size of the h_functions.
    """
    A = [[h1.subs({theta: th1, t: t1}) for h1 in h_functions]
         for th1, t1 in zip(atheta, time)]
    return A
