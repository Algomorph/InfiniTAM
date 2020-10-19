#!/usr/bin/python3

import subprocess
import argparse
import numpy as np
import os
import re
import sys
from mako.template import Template


# Requires Ubuntu for easy snap installations.
# Other platforms will require source code modifications, i.e. make sure that paths of
# meshlab.meshlabserver and CloudCompare below point to correct executables.
#
# Requires mako
# Requires Meshlab (Ubuntu: sudo snap install meshlab)
# Requires CloudCompare (Ubuntu: sudo snap install cloudcompare)

def roughly_align_and_prepare(input_mesh_path: str, output_mesh_path: str) -> int:
    script_path = os.path.abspath(__file__)
    mlx_path = os.path.join(os.path.dirname(script_path), "rough_alignment_and_normals_Snoopy.mlx")

    f = open(os.devnull, 'w')
    command_string = "/snap/bin/meshlab.meshlabserver -s " + mlx_path + " -i " + input_mesh_path + \
                     " -o " + output_mesh_path + " -m vn"
    print(command_string)
    output = subprocess.check_output(command_string.split(" "), stderr=f).decode()
    pattern = re.compile(r'\((\d+)\svn\s\d+\sfn\)')
    vertex_count = int(re.findall(pattern, output)[0])
    print(output)
    f.close()
    return vertex_count


def parse_registration_matrix(target_mesh_path: str) -> np.ndarray:
    target_mesh_directory = os.path.dirname(target_mesh_path)
    target_mesh_name = os.path.splitext(os.path.basename(target_mesh_path))[0]
    registration_matrix_prefix = target_mesh_name + "_" + "REGISTRATION_MATRIX"
    registration_matrix_files = [filename for filename in os.listdir(target_mesh_directory) if filename.startswith(registration_matrix_prefix)]
    if len(registration_matrix_files) == 0:
        raise ValueError("Target mesh directory, \"{:s}\", contains no registration matrix files.".format(target_mesh_directory))

    registration_matrix_files.sort(reverse=True)
    latest_registration_matrix_path = os.path.join(target_mesh_directory, registration_matrix_files[0])
    transformation_matrix = np.loadtxt(latest_registration_matrix_path, delimiter=" ")
    return transformation_matrix


def string_to_file(text, file_path):
    """Write a file with the given path and the given text."""
    output = open(file_path, "w")
    output.write(text)
    output.close()


def finely_align(roughly_aligned_mesh_path: str, output_mesh_path: str, fine_registration_matrix: np.ndarray) -> None:
    script_directory = os.path.dirname(os.path.abspath(__file__))
    templates_directory = "./templates"
    output_mesh_directory = os.path.dirname(output_mesh_path)
    output_mesh_name = os.path.splitext(os.path.basename(output_mesh_path))[0]
    # transformation_filter_template_path = os.path.join(script_directory, "meshlab_transformation_filter_mako_template.mlx")
    transformation_filter_template_path = "./meshlab_transformation_filter_mako_template.mlx"
    transformation_filter_template = Template(filename=transformation_filter_template_path, module_directory=templates_directory)
    transformation_filter_path = os.path.join(output_mesh_directory, output_mesh_name + "_meshlab_transformation_filter.mlx")

    string_to_file(transformation_filter_template.render(
        val0=fine_registration_matrix[0, 0],
        val1=fine_registration_matrix[0, 1],
        val2=fine_registration_matrix[0, 2],
        val3=fine_registration_matrix[0, 3],

        val4=fine_registration_matrix[1, 0],
        val5=fine_registration_matrix[1, 1],
        val6=fine_registration_matrix[1, 2],
        val7=fine_registration_matrix[1, 3],

        val8=fine_registration_matrix[2, 0],
        val9=fine_registration_matrix[2, 1],
        val10=fine_registration_matrix[2, 2],
        val11=fine_registration_matrix[2, 3],

        val12=fine_registration_matrix[3, 0],
        val13=fine_registration_matrix[3, 1],
        val14=fine_registration_matrix[3, 2],
        val15=fine_registration_matrix[3, 3],
    ), transformation_filter_path)

    command_string = "/snap/bin/meshlab.meshlabserver -s " + transformation_filter_path + " -i " + roughly_aligned_mesh_path + \
                     " -o " + output_mesh_path + " -m vn"
    f = open(os.devnull, 'w')
    output = subprocess.check_output(command_string.split(" "), stderr=f).decode()
    f.close()
    print(output)


def compute_fine_registration_matrix_using_icp(target_mesh_path: str, reference_mesh_path: str, vertex_count: int) -> np.ndarray:
    cloud_compare_environment = os.environ.copy()
    cloud_compare_environment["LD_LIBRARY_PATH"] = "/snap/cloudcompare/current/lib/cloudcompare"
    command_string = "/snap/cloudcompare/current/bin/CloudCompare -SILENT -AUTO_SAVE OFF " \
                     "-COMPUTE_NORMALS -o {:s} -o " \
                     "{:s} -ICP -MIN_ERROR_DIFF 1e-7 " \
                     "-RANDOM_SAMPLING_LIMIT {:d}".format(target_mesh_path, reference_mesh_path, vertex_count)
    f = open(os.devnull, 'w')
    output = subprocess.check_output(command_string.split(" "), stderr=f, env=cloud_compare_environment).decode()
    f.close()
    print(output)
    return parse_registration_matrix(target_mesh_path)


def align_two_meshes(input_mesh_path: str, reference_mesh_path: str, output_mesh_path: str) -> None:
    vertex_count = roughly_align_and_prepare(input_mesh_path, output_mesh_path)
    fine_transformation_matrix = compute_fine_registration_matrix_using_icp(output_mesh_path, reference_mesh_path, vertex_count)
    finely_align(output_mesh_path, output_mesh_path, fine_transformation_matrix)


PROGRAM_SUCCESS = 0
PROGRAM_FAIL = -1


def main() -> int:
    parser = argparse.ArgumentParser(description='Align Reco Snoopy output with Snoopy canonical mesh.')
    parser.add_argument('input_mesh_path', metavar='N', type=str,
                        help='mesh to transform')

    parser.add_argument('output_mesh_path', metavar='N', type=str,
                        help='path to output mesh, e.g. input mesh aligned to the reference mesh')

    parser.add_argument('reference_mesh_path', metavar='N', type=str,
                        help='mesh to transform')

    args = parser.parse_args()

    align_two_meshes(args.input_mesh_path, args.reference_mesh_path, args.output_mesh_path)
    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
