#  ================================================================
#  Created by Gregory Kramida on 2/7/20.
#  Copyright (c) 2020 Gregory Kramida
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

import sys
import numpy as np
import math

import pandas as pd

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


def get_hash_code(x, y, z):
    v1 = np.uint32(73856093)
    v2 = np.uint32(19349669)
    v3 = np.uint32(83492791)
    mask = np.uint32(0xFFFFF)

    return ((np.uint32(x) * v1) ^ (np.uint32(y) * v2) ^ (np.uint32(z) * v3)) & mask


def analyze(test_size):
    dim = math.floor(math.pow(test_size, 1.0 / 3.0))
    print("Real key count with test size", test_size, ":", dim * dim * dim)

    hash_hash = {}

    for x in range(0, dim):
        for y in range(0, dim):
            for z in range(0, dim):
                hash_code = np.uint32(get_hash_code(x, y, z))
                if hash_code in hash_hash:
                    hash_hash[hash_code] += 1
                else:
                    hash_hash[hash_code] = 1

    hash_table = np.zeros((len(hash_hash), 2), dtype=np.uint32)
    for (index, key) in enumerate(hash_hash):
        hash_table[index, 0] = key
        hash_table[index, 1] = hash_hash[key]

    df = pd.DataFrame(data=hash_table, columns=("hash", "count"))

    hash_count = len(hash_hash)
    print("Total hash count:", hash_count)

    def print_counts_above(above):
        counts_above = len(df[df["count"] > above])
        print("Counts above ", above, ": {:.2%}".format(counts_above / hash_count), ", or ",
              counts_above, "/", hash_count, ".", sep="")

    # print_counts_above(2)
    # print_counts_above(4)
    # print_counts_above(6)
    # print_counts_above(8)
    # print_counts_above(10)
    print_counts_above(15)
    df_slice = df[df["count"] > 15]
    df_slice.to_csv("colliding_hashes.csv", index=False)


def compile_hash_data_file(test_size):
    df = pd.read_csv("colliding_hashes.csv")
    hash_set = set(df["hash"].values)

    dim = math.floor(math.pow(test_size, 1.0 / 3.0))
    print("Real key count with test size", test_size, ":", dim * dim * dim)

    coord_by_hash = {}

    coords_per_hash = 16

    for x in range(0, dim):
        for y in range(0, dim):
            for z in range(0, dim):
                coord = [x, y, z]
                hash_code = np.uint32(get_hash_code(x, y, z))
                if hash_code in hash_set:
                    if hash_code not in coord_by_hash:
                        coord_by_hash[hash_code] = []
                    if len(coord_by_hash[hash_code]) < (coords_per_hash * 3):
                        coord_by_hash[hash_code] += coord

    coord_table = np.zeros((len(coord_by_hash), coords_per_hash * 3 + 1), dtype=np.uint32)
    for (index, key) in enumerate(coord_by_hash):
        coord_table[index] = [key] + coord_by_hash[key]

    df = pd.DataFrame(data=coord_table)
    df.to_csv(path_or_buf="test::snoopy.csv", columns=None, index=False)


def main():
    get_hash_code(23,139,64)
    # test_size = 10000000
    # analyze(test_size)
    # compile_hash_data_file(test_size)

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
