#!/usr/bin/python3

import subprocess
import argparse
import math
import os
import sys
import re

from collections import namedtuple

HausdorffDistance = namedtuple('HausdorffDistance', ['fileA', 'fileB', 'nptsA', 'dmin', 'dmax', 'dmean', 'dRMS'],
                               verbose=False)


# Requires Ubuntu for easy snap installations. Other platforms will require source code modifications, i.e. ensure
# that path of meshlab.meshlabserver points to the correct executable. Requires Meshlab (Ubuntu: sudo snap install
# meshlab)

def hausdorff_distance_one_direction(mesh1_filepath, mesh2_filepath):
    script_path = os.path.abspath(__file__)
    mlx_path = os.path.join(os.path.dirname(script_path), "distance.mlx")

    f = open(os.devnull, 'w')
    command_string = "/snap/bin/meshlab.meshlabserver -s " + mlx_path + " -i " + mesh1_filepath + " " + mesh2_filepath
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

    return HausdorffDistance(fileA=mesh1_filepath,
                             fileB=mesh2_filepath,
                             nptsA=point_count,
                             dmin=min_distance,
                             dmax=max_distance,
                             dmean=mean_distance,
                             dRMS=rms_distance)


def hausdorff_distance_bi(mesh1_filepath, mesh2_filepath):
    # get hausdorff dist from meshlab server                                                                                                                                                                
    hd_ab = hausdorff_distance_one_direction(mesh1_filepath, mesh2_filepath)
    hd_ba = hausdorff_distance_one_direction(mesh2_filepath, mesh1_filepath)

    min_bi = min(hd_ab.dmin, hd_ba.dmin)
    max_bi = max(hd_ab.dmax, hd_ba.dmax)

    sm = hd_ab.dmean * hd_ab.nptsA + hd_ba.dmean * hd_ba.nptsA
    mean_bi = sm / (hd_ab.nptsA + hd_ba.nptsA)

    ms = (hd_ab.dRMS ** 2) * hd_ab.nptsA + (hd_ba.dRMS ** 2) * hd_ba.nptsA
    rms_bi = math.sqrt(ms / (hd_ab.nptsA + hd_ba.nptsA))

    return HausdorffDistance(fileA=mesh1_filepath,
                             fileB=mesh2_filepath,
                             nptsA=hd_ab.nptsA,
                             dmin=min_bi,
                             dmax=max_bi,
                             dmean=mean_bi,
                             dRMS=rms_bi)


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

    dist = hausdorff_distance_bi(mesh_files[0], mesh_files[1])

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
