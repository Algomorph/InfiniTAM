import numpy as np


# ================= Analytical 3D Functions ====================================================#

class FunctionAndDerivatives3D:
    def __init__(self, name: str, function, analytical_first_derivative, analytical_second_derivative):
        self.name = name
        self.function = function
        self.analytical_first_derivative = analytical_first_derivative
        self.analytical_second_derivative = analytical_second_derivative


def function_3d_sin_sqrt(x, y):
    return np.sin(np.sqrt(x ** 2 + y ** 2))


def grad_3d_sin_sqrt(x, y):
    denominator = np.sqrt(x ** 2 + y ** 2)
    factor = np.cos(denominator) / denominator
    grad_x = x * factor
    grad_y = y * factor
    return np.dstack((grad_x, grad_y))


def hessian_3d_sin_sqrt(x, y):
    y_squared = y ** 2
    x_squared = x ** 2
    negative_product_x_y = -(y * x)
    sum_of_squares = x_squared + y_squared
    sum_of_squares_pow_3_over_2 = np.power(sum_of_squares, 3.0 / 2.0)
    sqrt_sum_of_squares = np.sqrt(x_squared + y_squared)
    cos_sqrt_sum_of_squares = np.cos(sqrt_sum_of_squares)
    sin_sqrt_sum_of_squares = np.sin(sqrt_sum_of_squares)
    factor1 = cos_sqrt_sum_of_squares / sum_of_squares_pow_3_over_2
    factor2 = sin_sqrt_sum_of_squares / sum_of_squares
    grad_xx = y_squared * factor1 - x_squared * factor2
    grad_yx = negative_product_x_y * (sqrt_sum_of_squares * sin_sqrt_sum_of_squares + cos_sqrt_sum_of_squares) / sum_of_squares_pow_3_over_2
    grad_xy = grad_yx.copy()
    grad_yy = x_squared * factor1 - y_squared * factor2
    hessian = np.dstack((grad_xx, grad_xy, grad_yx, grad_yy)).reshape((x.shape[0], x.shape[1], 2, 2))
    return hessian


# *** 3D sigmoid ***

def function_3d_sigmoid(x, y):
    return 1 / (1 + np.exp(-(x + y)))


def grad_3d_sigmoid(x, y):
    numerator = np.exp(-(x + y))
    denominator = (numerator + 1) ** 2
    grad_x = numerator / denominator
    grad_y = grad_x.copy()
    return np.dstack((grad_x, grad_y))


def hessian_3d_sigmoid(x, y):
    exponential_plus_one = np.exp(x + y) + 1
    grad_xx = -1 / exponential_plus_one + 3 / exponential_plus_one ** 2 - 2 / exponential_plus_one ** 3
    grad_xy = grad_xx.copy()
    grad_yx = grad_xx.copy()
    grad_yy = grad_xx.copy()
    hessian = np.dstack((grad_xx, grad_xy, grad_yx, grad_yy)).reshape((x.shape[0], x.shape[1], 2, 2))
    return hessian


sigmoid_3d = FunctionAndDerivatives3D("3D Sigmoid", function_3d_sigmoid, grad_3d_sigmoid, hessian_3d_sigmoid)
sin_sqrt_3d = FunctionAndDerivatives3D("sin(sqrt(x^2 + y^2))", function_3d_sin_sqrt, grad_3d_sin_sqrt, hessian_3d_sin_sqrt)
