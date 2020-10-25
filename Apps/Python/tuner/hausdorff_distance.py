#!/usr/bin/python3

import subprocess
import argparse
import math
import os
import sys
import re

from collections import namedtuple

HausdorffDistance = namedtuple('HausdorffDistance',
                               ['mesh1_path', 'mesh2_path', 'point_count_1', 'point_count_2',
                                'min_distance', 'max_distance',
                                'mean_distance', 'rms_distance'],
                               verbose=False)


# Requires Ubuntu for easy snap installations. Other platforms will require source code modifications, i.e. ensure
# that path of meshlab.meshlabserver points to the correct executable. Requires Meshlab (Ubuntu: sudo snap install
# meshlab)

def hausdorff_distance_unidirectional(mesh1_path: str, mesh2_path: str) -> HausdorffDistance:
    script_path = os.path.abspath(__file__)
    mlx_path = os.path.join(os.path.dirname(script_path), "distance.mlx")

    f = open(os.devnull, 'w')
    command_string = "/snap/bin/meshlab.meshlabserver -s " + mlx_path + " -i " + mesh1_path + " " + mesh2_path
    output = subprocess.check_output(command_string.split(" "), stderr=f)
    result = output.decode().split("\n")
    # parse result to get values                                                                                                                                                                         
    data = ""
    point_count = 0

    for idx, line in enumerate(result):
        m = re.match(r"\s*Sampled (\d+) pts.*", line)
        if m is not None:
            point_count = int(m.group(1))
        if line == 'Hausdorff Distance computed':
            data = result[idx + 2]
    m = re.match(r"\D+(\d+\.*\d*)\D+(\d+\.*\d*)\D+(\d+\.*\d*)\D+(\d+\.*\d*)", data)
    min_distance = float(m.group(1))
    max_distance = float(m.group(2))
    mean_distance = float(m.group(3))
    rms_distance = float(m.group(4))

    f.close()

    return HausdorffDistance(mesh1_path=mesh1_path,
                             mesh2_path=mesh2_path,
                             point_count_1=point_count,
                             point_count_2=-1,
                             min_distance=min_distance,
                             max_distance=max_distance,
                             mean_distance=mean_distance,
                             rms_distance=rms_distance)


def hausdorff_distance_bidirectional(mesh1_filepath, mesh2_filepath) -> HausdorffDistance:
    # get hausdorff dist from meshlab server                                                                                                                                                                
    hd_ab = hausdorff_distance_unidirectional(mesh1_filepath, mesh2_filepath)
    hd_ba = hausdorff_distance_unidirectional(mesh2_filepath, mesh1_filepath)

    min_bi = min(hd_ab.min_distance, hd_ba.min_distance)
    max_bi = max(hd_ab.max_distance, hd_ba.max_distance)

    sm = hd_ab.mean_distance * hd_ab.point_count_1 + hd_ba.mean_distance * hd_ba.point_count_1
    mean_bi = sm / (hd_ab.point_count_1 + hd_ba.point_count_1)

    ms = (hd_ab.rms_distance ** 2) * hd_ab.point_count_1 + (hd_ba.rms_distance ** 2) * hd_ba.point_count_1
    rms_bi = math.sqrt(ms / (hd_ab.point_count_1 + hd_ba.point_count_1))

    return HausdorffDistance(mesh1_path=mesh1_filepath,
                             mesh2_path=mesh2_filepath,
                             point_count_1=hd_ab.point_count_1,
                             point_count_2=hd_ba.point_count_1,
                             min_distance=min_bi,
                             max_distance=max_bi,
                             mean_distance=mean_bi,
                             rms_distance=rms_bi)


PROGRAM_SUCCESS = 0
PROGRAM_FAIL = -1


def main() -> int:
    parser = argparse.ArgumentParser(description='Compute Hausdorff Distance Between Two Meshes')
    parser.add_argument('input_mesh_path', metavar='N', type=str,
                        help='mesh to transform')

    parser.add_argument('output_mesh_path', metavar='N', type=str,
                        help='path to output mesh, e.g. input mesh aligned to the reference mesh')

    args = parser.parse_args()

    mesh_files = args.mesh_filepaths

    if len(mesh_files) != 2:
        print("wrong number of mesh files: wanted 2, got: " + str(len(mesh_files)))

    dist = hausdorff_distance_bidirectional(mesh_files[0], mesh_files[1])

    print(dist)

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
