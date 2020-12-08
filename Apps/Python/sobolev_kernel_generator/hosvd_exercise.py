#!/usr/bin/python3

import numpy as np
import tensorly.base as t_base
import tensorly.tenalg as t_alg
import tensorly.decomposition as t_decomp
import sys


def main():
    # X = tnsr in the tutorial
    X = np.moveaxis(np.array([[[1, 3],
                               [2, 4]],

                              [[5, 7],
                               [6, 8]],

                              [[9, 11],
                               [10, 12]]]), 0, 2)

    A = np.linalg.svd(t_base.unfold(X, 0))[0]  # output A matches tutorial
    print("A = \n", A)

    B = np.linalg.svd(t_base.unfold(X, 1))[0]  # output B matches tutorial
    print("B = \n", B)

    C = np.linalg.svd(t_base.unfold(X, 2))[0]  # output C matches tutorial
    print("C = \n", C)

    g = t_alg.mode_dot(t_alg.mode_dot(t_alg.mode_dot(X, A.T, 0), B.T, 1), C.T, 2)
    print("g = \n", g)

    g2 = t_decomp.tucker(X, ranks=(2, 2, 3)).core
    print("g2 = \n", g)

    return 0


if __name__ == "__main__":
    sys.exit(main())
