import math
import numpy as np
import scipy.optimize as opt
import sys

import hyperopt
from hyperopt import hp


def endpoint_loguniform(label, low, high):
    start_power = math.log(low)
    end_power = math.log(high)
    return hp.loguniform(label, start_power, end_power)


def f(x):  # The rosenbrock function
    return .5 * (1 - x[0]) ** 2 + (x[1] - x[0] ** 2) ** 2


def f2(x):
    return 3 * x[0] - x[0] ** 3 - 2 * x[1] ** 2 + x[1] ** 4


PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


def objective_function(args):
    x = args["x"]
    y = args["y"]
    loss = f((x, y))
    return {
        'loss': loss,
        'status': hyperopt.STATUS_OK
    }


def main() -> int:
    conjugate_gradient_result = opt.minimize(f, np.array([2, -1]), method="CG", options={'maxiter': 10})
    print(conjugate_gradient_result)
    print("minimum (CG): ", f(conjugate_gradient_result.x))
    parameter_space = {
        "x": endpoint_loguniform('x', 0.1, 2.0),
        "y": endpoint_loguniform('y', 0.1, 2.0)
    }
    trials = hyperopt.Trials()
    best_result = hyperopt.fmin(objective_function, parameter_space, algo=hyperopt.tpe.suggest,
                                max_evals=1000, trials=trials)

    print("Best tuning result:", best_result)
    print("minimum (TPE): ", f((best_result['x'], best_result['y'])))

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
