import numpy as np


# =============================== 2D Gradients =================================================#
def grad_central_differences_x(field: np.ndarray, grid_spacing: float = 1.0) -> np.array:
    grad_x = np.zeros_like(field)
    grad_x[:, 0] = field[:, 1] - field[:, 0]

    for i_x in range(1, field.shape[1] - 1):
        grad_x[:, i_x] = 0.5 * (field[:, i_x + 1] - field[:, i_x - 1]) / grid_spacing

    grad_x[:, field.shape[1] - 1] = field[:, field.shape[1] - 1] - field[:, field.shape[1] - 2]
    return grad_x


def grad_central_differences_y(field: np.ndarray, grid_spacing: float = 1.0) -> np.array:
    grad_y = np.zeros_like(field)
    grad_y[0, :] = field[1, :] - field[0, :]

    for i_x in range(1, field.shape[0] - 1):
        grad_y[i_x, :] = 0.5 * (field[i_x + 1, :] - field[i_x - 1, :]) / grid_spacing

    grad_y[field.shape[0] - 1, :] = field[field.shape[0] - 1, :] - field[field.shape[0] - 2, :]
    return grad_y


def grad_central_differences(field: np.ndarray, grid_spacing: float = 1.0) -> np.array:
    return np.dstack((grad_central_differences_x(field, grid_spacing), grad_central_differences_y(field, grid_spacing)))


def hessian_fd_local_fast(field: np.ndarray, x: int, y: int, grid_spacing: float = 1.0) -> np.ndarray:
    local_value = field[y, x]

    value_at_x_plus_one = field[y, x + 1]
    value_at_x_minus_one = field[y, x - 1]
    value_at_y_plus_one = field[y + 1, x]
    value_at_y_minus_one = field[y - 1, x]

    value_at_x_plus_one_y_plus_one = field[y + 1, x + 1]
    value_at_x_minus_one_y_minus_one = field[y - 1, x - 1]

    dxx = (value_at_x_plus_one - 2 * local_value + value_at_x_minus_one) / (grid_spacing ** 2)
    dyy = (value_at_y_plus_one - 2 * local_value + value_at_y_minus_one) / (grid_spacing ** 2)
    dxy = 0.5 * (value_at_x_plus_one_y_plus_one - value_at_x_plus_one -
                 value_at_y_plus_one + 2 * local_value - value_at_x_minus_one -
                 value_at_y_minus_one + value_at_x_minus_one_y_minus_one) / (grid_spacing ** 2)

    return np.array([[dxx, dxy],
                     [dxy, dyy]])


def hessian_fd_local_standard(field: np.ndarray, x: int, y: int, grid_spacing: float = 1.0) -> np.ndarray:
    local_value = field[y, x]

    value_at_x_plus_one = field[y, x + 1]
    value_at_x_minus_one = field[y, x - 1]
    value_at_y_plus_one = field[y + 1, x]
    value_at_y_minus_one = field[y - 1, x]

    value_at_x_plus_one_y_plus_one = field[y + 1, x + 1]
    value_at_x_plus_one_y_minus_one = field[y - 1, x + 1]
    value_at_x_minus_one_y_plus_one = field[y + 1, x - 1]
    value_at_x_minus_one_y_minus_one = field[y - 1, x - 1]

    dxx = (value_at_x_plus_one - 2 * local_value + value_at_x_minus_one) / (grid_spacing ** 2)
    dyy = (value_at_y_plus_one - 2 * local_value + value_at_y_minus_one) / (grid_spacing ** 2)
    dxy = 0.25 * (value_at_x_plus_one_y_plus_one - value_at_x_plus_one_y_minus_one -
                  value_at_x_minus_one_y_plus_one + value_at_x_minus_one_y_minus_one) / (grid_spacing ** 2)

    return np.array([[dxx, dxy],
                     [dxy, dyy]])


def hessian_fd_kernel_fast(field: np.ndarray, grid_spacing: float = 1.0) -> np.ndarray:
    padded_field = np.pad(field, (1,), 'edge')
    hessian = np.zeros((field.shape[0], field.shape[1], 2, 2), dtype=field.dtype)
    x_out = 0
    for x in range(1, padded_field.shape[1] - 1):
        y_out = 0
        for y in range(1, padded_field.shape[0] - 1):
            hessian[y_out, x_out] = hessian_fd_local_fast(padded_field, x, y, grid_spacing)
            y_out += 1
        x_out += 1
    return hessian


def hessian_fd_kernel_standard(field: np.ndarray, grid_spacing: float = 1.0) -> np.ndarray:
    padded_field = np.pad(field, (1,), 'edge')
    hessian = np.zeros((field.shape[0], field.shape[1], 2, 2))
    x_out = 0
    for x in range(1, padded_field.shape[1] - 1):
        y_out = 0
        for y in range(1, padded_field.shape[0] - 1):
            hessian[y_out, x_out] = hessian_fd_local_standard(padded_field, x, y, grid_spacing)
            y_out += 1
        x_out += 1
    return hessian


def hessian_fd_separable(field: np.ndarray, grid_spacing: float = 1.0) -> np.ndarray:
    padded_field = np.pad(field, (1,), 'edge')
    grad_x = grad_central_differences_x(padded_field, grid_spacing)
    grad_y = grad_central_differences_y(padded_field, grid_spacing)
    xx = grad_central_differences_x(grad_x, grid_spacing)
    xy = grad_central_differences_x(grad_y, grid_spacing)
    yx = grad_central_differences_y(grad_x, grid_spacing)
    yy = grad_central_differences_y(grad_y, grid_spacing)
    hessian = np.dstack((xx, xy, yx, yy)).reshape((padded_field.shape[0], padded_field.shape[1], 2, 2))[1:-1, 1:-1, :, :]
    return hessian
