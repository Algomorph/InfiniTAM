#!/usr/bin/python3
import pickle
import time
from collections import namedtuple
import argparse
import sys
import os
import math
import subprocess
import json
from enum import Enum

from typing import List, Optional, Dict, Any

import pandas as pd
import numpy as np
import switch_sets
import hyperopt
from hyperopt import hp

from hausdorff_distance import hausdorff_distance_bidirectional, HausdorffDistance
from mesh_alignment import align_two_meshes

switch_sets = {
    "Killing": switch_sets.level_set_evolution_switches_killing,
    "Killing_level_set": switch_sets.level_set_evolution_switches_killing_level_set,
    "Sobolev": switch_sets.level_set_evolution_switches_sobolev,
    "Tikhonov": switch_sets.level_set_evolution_switches_tikhonov
}

TunableParameter = namedtuple('TunableParameter',
                              ['expression', 'command_line_parameter'],
                              verbose=False)


class LossFunction(Enum):
    RMS_DISTANCE = 0
    MAX_DISTANCE = 1


def endpoint_loguniform(label, low, high):
    start_power = math.log(low)
    end_power = math.log(high)
    return hp.loguniform(label, start_power, end_power)


class Tuner:

    def __init__(self, reco_path, configuration_path, until_frame_index):
        self.reco_path = reco_path
        self.configuration_path = configuration_path
        self.configuration_json = self.get_configuration_as_json()
        self.output_path = self.determine_output_path()
        self.until_frame_index = until_frame_index
        self.last_frame_index = until_frame_index - 1
        self.base_command_string = self.compile_base_command_string()

    def compile_base_command_string(self):
        return self.reco_path + " --config=" + self.configuration_path + \
               " --automatic_run_settings.index_of_frame_to_end_before=" + str(self.until_frame_index) + \
               " --automatic_run_settings.save_meshes_after_processing=true" \
               " --logging_settings.log_benchmarks=false" + \
               " --logging_settings.log_to_disk=false --logging_settings.verbosity_level=top_level"

    def determine_output_path(self):
        output_path = self.configuration_json["paths"]["output_path"]
        if output_path == "<CONFIGURATION_DIRECTORY>":
            output_path = os.path.dirname(self.configuration_path)
        return output_path

    def get_configuration_as_json(self) -> dict:
        file = open(self.configuration_path, "r")
        configuration_text = "\n".join(file.readlines())
        file.close()
        return json.loads(configuration_text)

    @staticmethod
    def non_rigid_alignment_converged(output_log_lines: List[str]) -> bool:
        output_log_lines = str(output_log_lines).split("\\n")
        did_not_converge = len(output_log_lines) >= 3 and "did not converge" in output_log_lines[-3]
        return not did_not_converge

    def get_frame_folder(self, frame_index):
        return os.path.join(self.output_path, "Frame_{:03d}".format(frame_index))

    def run_command_and_process_output(self, command_string) -> Optional[HausdorffDistance]:
        dummy_error_file = open(os.devnull, 'w')
        print("")
        print("Running Reco with command: ")
        print(command_string)
        print("")
        output_log = subprocess.check_output(command_string.split(" "), stderr=dummy_error_file)

        if Tuner.non_rigid_alignment_converged(output_log):
            last_frame_folder = self.get_frame_folder(self.last_frame_index)
            canonical_mesh_path = os.path.join(last_frame_folder, "mesh.ply")
            aligned_mesh_path = os.path.join(last_frame_folder, "mesh_aligned.ply")
            reference_mesh_path = "SnoopyCanonical.ply"
            align_two_meshes(canonical_mesh_path, reference_mesh_path, aligned_mesh_path)
            dist = hausdorff_distance_bidirectional(aligned_mesh_path, reference_mesh_path)
            return dist
        return None

    def compile_command_string(self):
        return self.base_command_string

    def tune(self):
        tunable_parameters = {
            'learning_rate': TunableParameter(expression=endpoint_loguniform('learning_rate', 0.01, 0.5),
                                              command_line_parameter="--level_set_evolution.weights.learning_rate"),
            'warp_length_termination_threshold': TunableParameter(
                expression=hp.choice(
                    'warp_length_termination_threshold_type',
                    [
                        {
                            'warp_length_termination_threshold_type': 'maximum',
                            'update_length_threshold': endpoint_loguniform('maximum.update_length_threshold', 0.000010,
                                                                           0.000100)
                        },
                        {
                            'warp_length_termination_threshold_type': 'average',
                            'update_length_threshold': endpoint_loguniform('average.update_length_threshold', 0.000001,
                                                                           0.000010)
                        }
                    ]
                ),
                command_line_parameter=None),
            'update_length_threshold':
                TunableParameter(None, " --level_set_evolution.termination.update_length_threshold"),
            'warp_length_termination_threshold_type':
                TunableParameter(None, " --level_set_evolution.termination.warp_length_termination_threshold_type"),

        }

        parameter_space = {
            'switch_set': hp.choice('switch_set', [list(switch_sets.values())])
        }

        for name, tunable_parameter in tunable_parameters.items():
            if tunable_parameter.expression is not None: \
                    parameter_space[name] = tunable_parameter.expression

        def objective_function(args):
            print(args)

            start_time = time.time()
            command_string = self.compile_base_command_string()
            end_time = time.time()
            output_distance = self.run_command_and_process_output(command_string)
            loss = math.inf
            status = hyperopt.STATUS_FAIL

            if output_distance is not None:
                loss = output_distance.rms_distance
                status = hyperopt.STATUS_OK

            return {
                'loss': loss,
                'eval_time': end_time - start_time,
                'status': status,
                'attachments': dict(output_distance._asdict())
            }

        best_result = hyperopt.fmin(objective_function, parameter_space, algo=hyperopt.tpe.suggest, max_evals=1)
        print(best_result)
        return best_result


PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


def main() -> int:
    parser = argparse.ArgumentParser(description='Tune Reco parameters')
    parser.add_argument('reco_path', metavar='<reco_path>', type=str, help='Path to Reco executable')
    parser.add_argument('configuration_path', metavar='<configuration_path>', type=str,
                        help='Default configuration file path')
    parser.add_argument('until_frame', metavar='<until_frame>', type=int,
                        help='Run Reco until this frame (exclusive)')

    args = parser.parse_args()

    tuner = Tuner(args.reco_path, args.configuration_path, args.until_frame)
    tuner.tune()

    result_table = pd.DataFrame(None, columns=["level_set_term", "smoothing_term", "killing_field",
                                               "sobolev_gradient_smoothing", "learning_rate",
                                               "update_length_threshold"])

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
