#!/usr/bin/python3

import math
import sys
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import Apps.Python.finite_difference_tests.analytical_functions as af
import Apps.Python.finite_difference_tests.finite_differences as fd
from typing import List, Tuple

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


# see https://github.com/matplotlib/matplotlib/pull/11877
# and https://stackoverflow.com/questions/54994600/pyplot-legend-poly3dcollection-object-has-no-attribute-edgecolors2d
def surface_legend_hack(surface):
    surface._facecolors2d = surface._facecolors3d
    surface._edgecolors2d = surface._edgecolors3d


def generate_3d_subplot(ax, subplot_title, x_grid, y_grid, values_surface, label_surface, values_wireframe, label_wireframe):
    ax.set_title(subplot_title)
    analytical_grad_plot = ax.plot_surface(x_grid, y_grid, values_surface, rstride=1, cstride=1, cmap='winter', edgecolor='none',
                                           label=label_surface)
    surface_legend_hack(analytical_grad_plot)
    central_difference_plot = ax.plot_wireframe(x_grid, y_grid, values_wireframe, color='green',
                                                label=label_wireframe)

    ax.legend(handles=[central_difference_plot, analytical_grad_plot])


def root_mean_squared_error(analytical_solution, finite_difference_solution):
    cropped_fd = finite_difference_solution[1:-1, 1:-1]
    cropped_analytical = analytical_solution[1:-1, 1:-1]
    return math.sqrt(np.mean((cropped_analytical - cropped_fd) ** 2)) / np.mean(np.abs(analytical_solution))


def main():
    input_min = -6.0
    input_max = 6.0
    input_range = input_max - input_min
    grid_size = 30
    grid_spacing = input_range / grid_size

    x = np.linspace(input_min, input_max, grid_size)
    y = np.linspace(input_min, input_max, grid_size)
    x_grid, y_grid = np.meshgrid(x, y)

    # choose 3D function for all subsequent computations
    # function3d = af.sigmoid_3d
    # function3d = af.sin_sqrt_3d

    # *** configure figure & subplots
    fig = plt.figure()

    ax1 = fig.add_subplot(3, 3, 1, projection='3d')
    ax2 = fig.add_subplot(3, 3, 2, projection='3d')
    ax3 = fig.add_subplot(3, 3, 3, projection='3d')

    ax4 = fig.add_subplot(3, 3, 4, projection='3d')
    ax5 = fig.add_subplot(3, 3, 5, projection='3d')
    ax6 = fig.add_subplot(3, 3, 6, projection='3d')

    ax7 = fig.add_subplot(3, 3, 7, projection='3d')
    ax8 = fig.add_subplot(3, 3, 8, projection='3d')
    ax9 = fig.add_subplot(3, 3, 9, projection='3d')

    axes = (ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9)
    for ax in axes:
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

    # *** plot function
    z_values = function3d.function(x_grid, y_grid)

    ax1.set_title(function3d.name)
    ax1.plot_surface(x_grid, y_grid, z_values, rstride=1, cstride=1, cmap='winter', edgecolor='none')

    # *** plot gradient

    gradient_analytical = function3d.analytical_first_derivative(x_grid, y_grid)[:, :, 0]
    gradient_central_differences_x = fd.grad_central_differences_x(z_values, grid_spacing)
    gradient_central_differences_y = fd.grad_central_differences_y(z_values, grid_spacing)

    generate_3d_subplot(ax2, "Gradient by Central Differences (x)", x_grid, y_grid,
                        gradient_analytical, "analytical",
                        gradient_central_differences_x, "FD")

    generate_3d_subplot(ax3, "Gradient by Central Differences (y)", x_grid, y_grid,
                        gradient_analytical, "analytical",
                        gradient_central_differences_y, "FD")

    # *** plot Hessian based on separable finite differences
    hessian_analytical = function3d.analytical_second_derivative(x_grid, y_grid)
    hessian_analytical_xx = hessian_analytical[:, :, 0, 0]
    hessian_analytical_xy = hessian_analytical[:, :, 0, 1]
    hessian_analytical_yy = hessian_analytical[:, :, 1, 1]

    hessian_separable = fd.hessian_fd_separable(z_values, grid_spacing)
    hessian_separable_xx = hessian_separable[:, :, 0, 0]

    generate_3d_subplot(ax4, "Hessian by Separable FD (xx)", x_grid, y_grid,
                        hessian_analytical_xx, "analytical",
                        hessian_separable_xx, "FD")

    hessian_separable_xy = hessian_separable[:, :, 0, 1]

    generate_3d_subplot(ax5, "Hessian by Separable FD (xy)", x_grid, y_grid,
                        hessian_analytical_xy, "analytical",
                        hessian_separable_xy, "FD")

    hessian_separable_yy = hessian_separable[:, :, 1, 1]

    generate_3d_subplot(ax6, "Hessian by Separable FD (yy)", x_grid, y_grid,
                        hessian_analytical_yy, "analytical",
                        hessian_separable_yy, "FD")

    # *** plot Hessian based on kernel finite differences
    hessian_kernel = fd.hessian_fd_kernel_fast(z_values, grid_spacing)
    hessian_kernel_xx = hessian_kernel[:, :, 0, 0]

    generate_3d_subplot(ax7, "Hessian by Kernel FD (xx)", x_grid, y_grid,
                        hessian_analytical_xx, "analytical",
                        hessian_kernel_xx, "FD")

    hessian_kernel_xy = hessian_kernel[:, :, 0, 1]

    generate_3d_subplot(ax8, "Hessian by Kernel FD (xy)", x_grid, y_grid,
                        hessian_analytical_xy, "analytical",
                        hessian_kernel_xy, "FD")

    hessian_kernel_yy = hessian_kernel[:, :, 1, 1]

    generate_3d_subplot(ax9, "Hessian by Kernel FD (yy)", x_grid, y_grid,
                        hessian_analytical_yy, "analytical",
                        hessian_kernel_yy, "FD")

    # *** compute and print mean errors ***
    error_separable = root_mean_squared_error(hessian_analytical, hessian_separable)
    error_kernel = root_mean_squared_error(hessian_analytical, hessian_kernel)
    fig.suptitle("RMSE for separable FD (1): {:.3%}. RMSE for 2D kernel FD (2): {:.3%}. (2) / (1): {:.3%}"
                 .format(error_separable, error_kernel, error_kernel / error_separable))

    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    plt.show()
    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
