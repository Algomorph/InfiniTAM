#!/usr/bin/python3

from hausdorff_distance import hausdorff_distance_bi
from mesh_alignment import align_two_meshes

import argparse
import sys
import os
import subprocess
import json

from typing import List

PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


class NumericRange:
    def __init__(self, start, end):
        self.start = start
        self.end = end


tunable_parameters = {
    "--level_set_evolution.weights.learning_rate": NumericRange(0.01, 0.5)
}


def non_rigid_alignment_converged(output_log_lines: List[str]) -> bool:
    output_log_lines = str(output_log_lines).split("\\n")
    did_not_converge = len(output_log_lines) >= 3 and "did not converge" in output_log_lines[-3]
    return not did_not_converge


def get_configuration_as_json(configuration_path: str) -> dict:
    file = open(configuration_path, "r")
    configuration_text = "\n".join(file.readlines())
    file.close()
    return json.loads(configuration_text)


def determine_output_path(configuration: dict, configuration_path: str):
    output_path = configuration["paths"]["output_path"]
    print(output_path)
    if output_path == "<CONFIGURATION_DIRECTORY>":
        output_path = os.path.dirname(configuration_path)
    return output_path


def get_frame_folder(output_path, frame_index):
    return os.path.join(output_path, "Frame_{:03d}".format(frame_index))


def main() -> int:
    parser = argparse.ArgumentParser(description='Tune Reco parameters')
    parser.add_argument('reco_path', metavar='<reco_path>', type=str, help='Path to Reco executable')
    parser.add_argument('configuration_path', metavar='<configuration_path>', type=str,
                        help='Default configuration file path')
    parser.add_argument('until_frame', metavar='<until_frame>', type=int,
                        help='Run Reco until this frame (exclusive)')

    args = parser.parse_args()
    last_frame = args.until_frame - 1
    configuration = get_configuration_as_json(args.configuration_path)
    output_path = determine_output_path(configuration, args.configuration_path)

    command_string = args.reco_path + " --config=" + args.configuration_path + \
                     " --automatic_run_settings.index_of_frame_to_end_before=" + str(args.until_frame) + \
                     " --automatic_run_settings.save_meshes_after_processing=true" \
                     " --logging_settings.log_benchmarks=false" + \
                     " --logging_settings.log_to_disk=false --logging_settings.verbosity_level=top_level"
    dummy_error_file = open(os.devnull, 'w')
    print("")
    print("Running Reco with command: ")
    print(command_string)
    print("")
    output_log = subprocess.check_output(command_string.split(" "), stderr=dummy_error_file)

    if non_rigid_alignment_converged(output_log):
        last_frame_folder = get_frame_folder(output_path, last_frame);
        canonical_mesh_path = os.path.join(last_frame_folder, "mesh.ply")
        aligned_mesh_path = os.path.join(last_frame_folder, "mesh_aligned.ply")
        reference_mesh_path = "SnoopyCanonical.ply"
        align_two_meshes(canonical_mesh_path, aligned_mesh_path, reference_mesh_path)
        dist = hausdorff_distance_bi(aligned_mesh_path, reference_mesh_path)
        print(dist)

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
