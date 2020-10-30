#!/usr/bin/python3
#  ================================================================
#  Created by Gregory Kramida on 2/1/18.
#  Copyright (c) 2018 Gregory Kramida
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#  ================================================================

from __future__ import print_function
import sys
from sktensor import dtensor
from math_utils import tenmat, tucker

import numpy as np
import argparse as ap
import sys
import scipy.io

EXIT_STATUS_SUCCESS = 0
EXIT_STATUS_FAILURE = 1


def print_to_stderr(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def get_voxel_coord(index, s):
    """
    Based on provided integer voxel index and lateral size of the 3D neighborhood, determines the coordinates of the
    voxel in the neighborhood
    :type index: int
    :param index:
    :type s: int
    :param s:
    :return:
    """
    s_squared = s ** 2
    z = index // s_squared
    remainder = index - (z * s_squared)
    y = remainder // s
    x = remainder - (y * s)
    return x, y, z


def get_voxel_index(coord, s):
    """
    Computes voxel index in an s x s x s neighborhood based on the provide coordinates
    Note: returned index may be negative or > (s * s * s) if the provided index coordinate corresponds to a voxel at the
    boundary of the s x s x s block
    :type coord: tuple[int]
    :param coord: tuple containing (x, y, z) coordinates of the voxel
    :type s: int
    :param s: lateral size of the neighborhood
    :return: index of the voxel at the specified coordinate
    """
    x, y, z = coord
    return s * s * z + s * y + x


def get_voxel_six_connected_neighbor_indices(voxel_index, s):
    """
    Get indices of all six-connected neighbors of the specified voxel based on an s x s x s neighborhood.
    These indices might be negative or greater than (s*s*s), see signature of get_voxel_index for details.
    :type voxel_index: int
    :param voxel_index: index of the central voxel
    :param s: lateral size of the neighborhood
    :return: indices of the six-connected neighbors
    """
    x, y, z = get_voxel_coord(voxel_index, s)
    index0 = get_voxel_index((x - 1, y, z), s)
    index1 = get_voxel_index((x + 1, y, z), s)
    index2 = get_voxel_index((x, y + 1, z), s)
    index3 = get_voxel_index((x, y - 1, z), s)
    index4 = get_voxel_index((x, y, z - 1), s)
    index5 = get_voxel_index((x, y, z + 1), s)
    return [index0, index1, index2, index3, index4, index5]


def is_six_connected_neighbor(coord_first, coord_second):
    """
    Determines if the voxel with the second coordinates is a six-connected neighbor of the voxel
    with the first coordinates. Note: Assumes voxel coordinates are integers.
    :type coord_first: tuple[int]
    :param coord_first: first coordinate set
    :type coord_second: tuple[int]
    :param coord_second: second coordinate set
    :return: whether the voxel with the second coordinates is a six-connected neighbor of the voxel
    with the first coordinates
    """
    (x1, y1, z1) = coord_first
    (x2, y2, z2) = coord_second
    dist_x = abs(x2 - x1)
    dist_y = abs(y2 - y1)
    dist_z = abs(z2 - z1)
    return dist_x + dist_y + dist_z == 1


def generate_7pt_stencil_finite_difference_laplacian_matrix(s, precision):
    """
    Generates an s^3-by-s^3 matrix, where:
      1. Each row & represents one of the s*s*s voxels composing the Sobolev kernel.
      2. Each column represents another voxel in the kernel.
      3. Ordering is by x, y, and then the z axis, in that order.
      4. Each entry represents the value of the Laplacian operator based on the neighborhood relationship between
         the voxel represented by the current row and the voxel represented by the current column
    :type s: int
    :param s: lateral size of the kernel
    :type precision: type
    :param precision: numpy precision
    :return:
    """
    s_cubed = s ** 3

    stencil_laplacian_operator = np.zeros((s_cubed, s_cubed), precision)
    for voxel_ix in range(0, s_cubed):
        voxel_coord = get_voxel_coord(voxel_ix, s)
        for neighbor_ix in range(0, s_cubed):
            if voxel_ix == neighbor_ix:
                # voxel & self, center value of the 3D laplacian operator
                stencil_laplacian_operator[voxel_ix, neighbor_ix] = -6
            other_coord = get_voxel_coord(neighbor_ix, s)
            if is_six_connected_neighbor(voxel_coord, other_coord):
                stencil_laplacian_operator[voxel_ix, neighbor_ix] = 1
    return stencil_laplacian_operator


def generate_7pt_stencil_finite_difference_laplacian_matrix2(s, precision):
    """
    Generates an s^3-by-s^3 matrix, where:
      1. Each row & represents one of the s*s*s voxels composing the Sobolev kernel.
      2. Each column represents another voxel in the kernel.
      3. Ordering is by x, y, and then the z axis, in that order.
      4. Each entry represents the value of the Laplacian operator based on the neighborhood relationship between
         the voxel represented by the current row and the voxel represented by the current column
    :type s: int
    :param s: lateral size of the kernel
    :type precision: type
    :param precision: numpy precision
    :return:
    """
    s_cubed = s ** 3

    stencil_laplacian_operator = np.zeros((s_cubed, s_cubed), precision)
    for voxel_ix in range(0, s_cubed):
        six_connected_neighbor_indices = get_voxel_six_connected_neighbor_indices(voxel_ix, s)
        stencil_laplacian_operator[voxel_ix, voxel_ix] = -6.0
        for neighbor_ix in six_connected_neighbor_indices:
            if 0 <= neighbor_ix < s_cubed:
                stencil_laplacian_operator[voxel_ix, neighbor_ix] = 1.0
    return stencil_laplacian_operator


def args_to_parameters():
    parser = ap.ArgumentParser("Generates Sobolev separable 1D filters (approximations) for given s (filter size) and "
                               "lambda (filter strength) parameters")
    parser.add_argument("-s", "--size", type=int, default=7,
                        help="Size of the filter. Sizes 3 and below were shown not to be sufficient; sizes larger than"
                             " 7 impede speed, with negligible performance improvement.")
    parser.add_argument("-l", "--lambda_", type=float, default=0.1,
                        help="Strength of the filter, between 0.0 and 1.0. A good default value is 0.1. "
                             "Doubling it was experimentally shown to decrease the number of iterations by 3-8%.")
    parser.add_argument("-p", "--precision", type=int, default=32,
                        help="Floating-point precision to use. May be 32 or 64.")
    parser.add_argument("-alm", '--alternative_laplacian_method', dest='alternative_laplacian_method',
                        action='store_true')
    parser.add_argument("-dlm", '--default_laplacian_method', dest='alternative_laplacian_method', action='store_false')
    parser.set_defaults(alternative_laplacian_method=False)
    parser.add_argument("-sar", "--use_s_as_rank", dest='use_s_as_rank', action='store_true')
    parser.set_defaults(use_s_as_rank=False)

    parameters = parser.parse_args()

    s = parameters.size
    l = parameters.lambda_
    p = parameters.precision

    if s < 1 or s > 256:
        print_to_stderr("Filter size s must be an integer in [1,256]")
        parser.print_help()
        return None

    if l < 0.0 or l > 1.0:
        print_to_stderr("Filter strength lambda must be a real number in [0,1.0]")
        parser.print_help()
        return None

    if p not in [32, 64]:
        print_to_stderr("Precision must be in ")
        parser.print_help()
        return None
    if p == 32:
        precision = np.float32
    else:
        precision = np.float64
    parameters.precision = precision
    return parameters


def generate_1d_sobolev_kernel(size=7, strength=0.1, precision=np.float32,
                               alternative_laplacian_method=False, use_size_as_rank=False, return_u_matrices=False):
    s_cubed = size ** 3
    # For documentation / reference only
    # laplacian_3d_operator = np.array([[[0, 0, 0],
    #                                    [0, 1, 0],
    #                                    [0, 0, 0]],
    #                                   [[0, 1, 0],
    #                                    [1, -6, 1],
    #                                    [0, 1, 0]],
    #                                   [[0, 0, 0],
    #                                    [0, 1, 0],
    #                                    [0, 0, 0]]], dtype=np.int32)

    # (Id - l*delta)S = v

    identity_matrix = np.identity(s_cubed, precision)
    # stencil_laplacian_operator  = delta
    # Each row & represents one of the s*s*s voxels composing the Sobolev kernel.
    # Each column represents another voxel in the kernel.
    # Ordering is assumed to be by x, y, and then z axis, in that order.
    if alternative_laplacian_method:
        stencil_laplacian_operator = generate_7pt_stencil_finite_difference_laplacian_matrix(size, precision)
    else:
        stencil_laplacian_operator = generate_7pt_stencil_finite_difference_laplacian_matrix2(size, precision)
    # one_hot_vector = v
    one_hot_vector = np.zeros((s_cubed, 1), precision)
    one_hot_vector[s_cubed // 2] = 1.0

    # solve the system for S
    sobolev_kernel_flat = np.linalg.solve((identity_matrix - strength * stencil_laplacian_operator),
                                          one_hot_vector)
    u, v, vh = np.linalg.svd(sobolev_kernel_flat)
    sobolev_kernel = sobolev_kernel_flat.reshape((size, size, size))
    sobolev_kernel_tensor = dtensor(sobolev_kernel)

    if use_size_as_rank:
        core, u_matrices = tucker.hooi(sobolev_kernel_tensor, size)
    else:
        core, u_matrices = tucker.hooi(sobolev_kernel_tensor)

    if return_u_matrices:
        return u_matrices
    else:
        return -u_matrices[1][:, 0]


def main():
    parameters = args_to_parameters()
    if parameters is None:
        return EXIT_STATUS_FAILURE

    s = parameters.size
    l = parameters.lambda_
    precision = parameters.precision
    u_matrices = generate_1d_sobolev_kernel(s, l, precision, parameters.alternative_laplacian_method,
                                            parameters.use_s_as_rank, return_u_matrices=True)
    [u1, u2, u3] = u_matrices

    # scipy.io.savemat("sob.mat", dict(sob=sobolev_kernel))
    print("=====================================")
    print("U matrices for x, y, and z direction:")
    print("=====================================")
    print("U1:")
    print(u1)
    print("U2:")
    print(u2)
    print("U3:")
    print(u3)
    print()
    print("% average difference between U1 and U2: ", np.abs(u1 - u2).mean() * 100)
    print("% average difference between U2 and U3: ", np.abs(u2 - u3).mean() * 100)
    print("% average difference between U1 and U3: ", np.abs(u1 - u3).mean() * 100)

    filters = []
    for u in u_matrices:
        filters.append((u[:, 0]).reshape((-1, 1)))

    print("=====================================")
    print("1D Filters (columns):")
    print("=====================================")
    print(np.hstack(filters))

    np.savetxt("filter.csv", -filters[0], delimiter=",")

    return EXIT_STATUS_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
