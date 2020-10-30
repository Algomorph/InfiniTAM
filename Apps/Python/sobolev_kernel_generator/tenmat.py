#  ================================================================
#  Created by Gregory Kramida on 4/24/18.
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

# !/usr/bin/env python

import numpy as np
from sktensor.core import tensor_mixin
from sktensor import dtensor
from numbers import Number


# import tools

def get_elements_at(nda, indices):
    """
    From the given nda(Iterable, e.g. ndarray, list, or tuple),
    returns the list located at the given indices
    """
    ret = []
    for i in indices:
        ret.append(nda[i])
    return np.array(ret)


def arange_sans(n, vector):
    """
    returns a numpy.array object that contains
    elements in [0,1, ... n-1] but not in vector.
    """
    ret = list(np.arange(n))
    for i in vector:
        if 0 <= i < n:
            ret.remove(i)
    return np.array(ret)


# TODO: extend numpy.ndarray instead of encapsulating it in .data to simplify usage
class tenmat():
    def __init__(self, T, row_wise_dimensions=None, column_wise_dimensions=None, original_tensor_size=None,
                 options=None):

        if row_wise_dimensions is not None and row_wise_dimensions.__class__ == list:
            row_wise_dimensions = np.array(row_wise_dimensions)
        if column_wise_dimensions is not None and column_wise_dimensions.__class__ == list:
            column_wise_dimensions = np.array(column_wise_dimensions)
        if original_tensor_size is not None and original_tensor_size.__class__ == list:
            original_tensor_size = np.array(original_tensor_size)

        # constructor for the call tenmat(A, RDIMS, CDIMS, TSIZE)
        if row_wise_dimensions is not None and column_wise_dimensions is not None and original_tensor_size is not None:
            if T.__class__ == np.ndarray or T.__class__ == tensor_mixin:
                self.data = T.copy()
            self.row_indices = row_wise_dimensions
            self.column_indices = column_wise_dimensions
            self.original_tensor_size = tuple(original_tensor_size)
            n = len(self.original_tensor_size)

            temp = np.concatenate((self.row_indices, self.column_indices))
            temp.sort()
            if not ((np.arange(n) == temp).all()):
                raise ValueError("Incorrect specification of dimensions")
            elif get_elements_at(self.original_tensor_size, self.row_indices).prod() != len(self.data):
                raise ValueError("size(T,0) does not match size specified")

            elif get_elements_at(self.original_tensor_size, self.column_indices).prod() != len(self.data[0]):
                raise ValueError("size(T,1) does not match size specified")

        # convert tensor to a tenmat
        if row_wise_dimensions is None and column_wise_dimensions is None:
            raise ValueError("Both of rdim and cdim are not given");

        T = T.copy()  # copy the tensor
        self.original_tensor_size = T.shape
        n = T.ndim

        if isinstance(row_wise_dimensions, Number):
            row_wise_dimensions = [row_wise_dimensions]
        if isinstance(row_wise_dimensions, Number):
            column_wise_dimensions = [column_wise_dimensions]

        if row_wise_dimensions is not None:
            if column_wise_dimensions is not None:
                rdims = row_wise_dimensions
                cdims = column_wise_dimensions
            elif options is not None:
                if options == 'fc':
                    rdims = row_wise_dimensions
                    if rdims.size != 1:
                        raise ValueError("only one row dimension for 'fc' option");
                    cdims = []
                    for i in range(row_wise_dimensions[0] + 1, n):
                        cdims.append(i)
                    for i in range(0, row_wise_dimensions[0]):
                        cdims.append(i)
                    cdims = np.array(cdims)
                elif options == 'bc':
                    rdims = row_wise_dimensions
                    if rdims.size != 1:
                        raise ValueError("only one row dimension for 'bc' option");
                    cdims = []
                    for i in range(0, row_wise_dimensions[0])[::-1]:
                        cdims.append(i)
                    for i in range(row_wise_dimensions[0] + 1, n)[::-1]:
                        cdims.append(i)
                    cdims = np.array(cdims)

                else:
                    raise ValueError("unknown option {0}".format(options));

            else:
                rdims = row_wise_dimensions
                cdims = arange_sans(n, rdims)

        else:
            if options == 't':
                cdims = column_wise_dimensions
                rdims = arange_sans(n, cdims)
            else:
                raise ValueError("unknown option {0}".format(options));

        # error check
        temp = np.concatenate((rdims, cdims))
        temp.sort()
        if not ((np.arange(n) == temp).all()):
            raise ValueError("error, Incorrect specification of dimensions");

        # permute T so that the dimensions specified by RDIMS come first

        # !!!! order of data in ndarray is different from that in Matlab!
        # this is (kind of odd process) needed to conform the result with Matlab!
        # lis = list(T.shape);
        # temp = lis[T.ndims()-1];
        # lis[T.ndims()-1] = lis[T.ndims()-2];
        # lis[T.ndims()-2] = temp;
        # T.data = T.data.reshape(lis).swapaxes(T.ndims()-1, T.ndims()-2);
        # print T;

        # T = T.permute([T.ndims()-1, T.ndims()-2]+(range(0,T.ndims()-2)));
        # print T;
        T = T.transpose(np.concatenate((rdims, cdims)))
        # convert T to a matrix;

        row = get_elements_at(self.original_tensor_size, rdims).prod()
        col = get_elements_at(self.original_tensor_size, cdims).prod()

        self.data = T.reshape([row, col])
        self.row_indices = rdims
        self.column_indices = cdims

    def copy(self, **kwargs):
        return tenmat(self.data, self.row_indices, self.column_indices, self.original_tensor_size)

    def as_dtensor(self):
        sz = self.original_tensor_size
        order = np.concatenate((self.row_indices, self.column_indices))
        order = order.tolist()
        data = self.data.reshape(get_elements_at(sz, order))
        # transpose + argsort(order) equals ipermute
        data = data.transpose(np.argsort(order))
        return dtensor(data)

    def as_ndarray(self):
        """returns an ndarray(2-D) that contains the same value with the tenmat"""
        return self.data

    def __str__(self):
        ret = ""
        ret += "matrix corresponding to a tensor of size {0}\n".format(self.original_tensor_size)
        ret += "rindices {0}\n".format(self.row_indices)
        ret += "cindices {0}\n".format(self.column_indices)
        ret += "{0}\n".format(self.data)
        return ret
