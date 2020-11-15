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


class TunableParameter:
    def __init__(self, expression, command_line_parameter):
        self.expression = expression
        self.command_line_parameter = command_line_parameter

    def cmd(self, value):
        if self.command_line_parameter is None:
            return ""
        else:
            return "--" + self.command_line_parameter + "=" + str(value)


class Mode(Enum):
    NORMAL = 1
    TEST_2_PARAMETERS = 2
    TEST_1_PARAMETER = 3


class LossFunction(Enum):
    RMS_DISTANCE = 0
    MAX_DISTANCE = 1


def endpoint_loguniform(label, low, high):
    start_power = math.log(low)
    end_power = math.log(high)
    return hp.loguniform(label, start_power, end_power)


voxel_size_list = [0.003, 0.004, 0.005, 0.006]
truncation_distance_factor_list = [8, 10]
switch_set_names = ["Killing", "Killing_level_set", "Sobolev", "Tikhonov"]
warp_length_termination_threshold_type_names = ['maximum', 'average']

tunable_parameters = {
    'voxel_size': TunableParameter(expression=hp.choice('voxel_size', voxel_size_list),
                                   command_line_parameter="general_voxel_volume_parameters.voxel_size"),
    'truncation_distance_factor': TunableParameter(
        expression=hp.choice('truncation_distance_factor', truncation_distance_factor_list),
        command_line_parameter=None),
    'truncation_distance':
        TunableParameter(None, "general_voxel_volume_parameters.truncation_distance"),
    'learning_rate': TunableParameter(expression=endpoint_loguniform('learning_rate', 0.005, 0.5),
                                      command_line_parameter="level_set_evolution.weights.learning_rate"),
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
        TunableParameter(None, "level_set_evolution.termination.update_length_threshold"),
    'warp_length_termination_threshold_type':
        TunableParameter(None, "level_set_evolution.termination.warp_length_termination_threshold_type"),
    'switch_set':
        TunableParameter(
            expression=hp.choice(
                "switch_set",
                [
                    {
                        "switches": switch_sets.level_set_evolution_switches_killing,
                        "Killing_dampening_factor": hp.uniform("Killing.Killing_dampening_factor", 0.05, 0.8),
                        "weight_smoothing_term": hp.uniform("Killing.weight_smoothing_term", 0.05, 1.0)
                    },
                    {
                        "switches": switch_sets.level_set_evolution_switches_killing_level_set,
                        "Killing_dampening_factor": hp.uniform("Killing_level_set.Killing_dampening_factor", 0.05, 0.8),
                        "weight_smoothing_term": hp.uniform("Killing_level_set.weight_smoothing_term", 0.05, 1.0),
                        "weight_level_set_term": hp.uniform("Killing_level_set.weight_level_set_term", 0.05, 0.4)
                    },
                    {
                        "switches": switch_sets.level_set_evolution_switches_tikhonov,
                        "weight_smoothing_term": hp.uniform("Tikhonov.weight_smoothing_term", 0.05, 1.0)
                    },
                    {
                        "switches": switch_sets.level_set_evolution_switches_sobolev,
                        "weight_smoothing_term": hp.uniform("Sobolev.weight_smoothing_term", 0.05, 1.0)
                    },
                ]
            ),
            command_line_parameter=None),
    'Killing_dampening_factor': TunableParameter(None, "level_set_evolution.weights.Killing_dampening_factor"),
    'weight_smoothing_term': TunableParameter(None, "level_set_evolution.weights.weight_smoothing_term"),
    'weight_level_set_term': TunableParameter(None, "level_set_evolution.weights.weight_level_set_term"),

}

two_parameter_test_subset = {
    "learning_rate": tunable_parameters["learning_rate"],
    "update_length_threshold":
        TunableParameter(endpoint_loguniform('maximum.update_length_threshold', 0.000010, 0.000100),
                         "level_set_evolution.termination.update_length_threshold")
}

one_parameter_test_subset = {
    "learning_rate": tunable_parameters["learning_rate"]
}


class Tuner:

    def __init__(self, reco_path: str, configuration_path: str, until_frame_index: int, loss_function: LossFunction,
                 trials_step: int, verbose: bool, mode: Mode):
        self.reco_path = reco_path
        self.configuration_path = configuration_path
        self.configuration_json = self.get_configuration_as_json()
        self.output_path = self.determine_output_path()
        self.until_frame_index = until_frame_index
        self.last_frame_index = until_frame_index - 1
        self.base_command_string = self.compile_base_command_string()
        self.loss_function = loss_function
        self.trials_step = trials_step
        self.verbose = verbose
        self.mode = mode

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
        did_not_converge = len(output_log_lines) >= 3 and "did not converge" in output_log_lines[-2]
        return not did_not_converge

    def get_frame_folder(self, frame_index):
        return os.path.join(self.output_path, "Frame_{:03d}".format(frame_index))

    def run_command_and_process_output(self, command_string) -> Optional[HausdorffDistance]:
        print("")
        print("Running Reco with command: ")
        print(command_string)
        print("")
        try:
            output_log_lines = subprocess.check_output(command_string.split(" "), encoding='ASCII').split("\n")
            if Tuner.non_rigid_alignment_converged(output_log_lines):
                last_frame_folder = self.get_frame_folder(self.last_frame_index)
                canonical_mesh_path = os.path.join(last_frame_folder, "mesh.ply")
                aligned_mesh_path = os.path.join(last_frame_folder, "mesh_aligned.ply")
                reference_mesh_path = "SnoopyCanonical.ply"
                align_two_meshes(canonical_mesh_path, reference_mesh_path, aligned_mesh_path, self.verbose)
                distance = hausdorff_distance_bidirectional(aligned_mesh_path, reference_mesh_path)
                return distance
        except subprocess.CalledProcessError as e:
            print(e.output)
            print(e)
        return None

    @staticmethod
    def __compile_command_line_representation(command, args):
        for name, value in args.items():
            if name in tunable_parameters and tunable_parameters[name].command_line_parameter is not None:
                command += " " + tunable_parameters[name].cmd(value)
            elif type(value) is dict:
                command += Tuner.__compile_command_line_representation(command, value)
        return command

    def compile_command_string(self, args):

        command = self.base_command_string

        if "switch_set" in args:
            switches = args["switch_set"]["switches"]
            command += switches.parameters.to_command_line_string()

        for name, value in args.items():
            if name in tunable_parameters and tunable_parameters[name].command_line_parameter is not None:
                command += " " + tunable_parameters[name].cmd(value)
            elif type(value) is dict:
                command = Tuner.__compile_command_line_representation(command, value)

        if "truncation_distance_factor" in args:
            narrow_band_half_width = args["truncation_distance_factor"] * args["voxel_size"]
            command += " " + tunable_parameters["truncation_distance"].cmd(narrow_band_half_width)

        return command

    def tune(self):
        parameter_space = {}

        if self.mode == Mode.NORMAL:
            parameters_to_tune = tunable_parameters
        elif self.mode == Mode.TEST_1_PARAMETER:
            parameters_to_tune = one_parameter_test_subset
        elif self.mode == Mode.TEST_2_PARAMETERS:
            parameters_to_tune = two_parameter_test_subset
        else:
            raise ValueError("Unsupported mode: " + str(self.mode))

        for name, tunable_parameter in parameters_to_tune.items():
            if tunable_parameter.expression is not None:
                parameter_space[name] = tunable_parameter.expression

        def objective_function(args):
            start_time = time.time()
            command_string = self.compile_command_string(args)
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
                'attachments': {} if output_distance is None else dict(output_distance._asdict())
            }

        trials_step = self.trials_step

        try:  # try to load an already saved trials object, and increase the max
            trials = pickle.load(open("tuning_table.hyperopt", "rb"))
            print("Found saved Trials! Loading...")
        except:  # create a new trials object and start searching
            trials = hyperopt.Trials()

        max_trials = len(trials.trials) + trials_step
        print("Rerunning from {} trials to {} (+{}) trials".format(len(trials.trials), max_trials, trials_step))

        best_result = hyperopt.fmin(objective_function, parameter_space, algo=hyperopt.tpe.suggest,
                                    max_evals=max_trials, trials=trials)

        print("Best tuning result:", best_result)

        # save the trials object
        with open("tuning_table.hyperopt", "wb") as f:
            pickle.dump(trials, f)

        return best_result


PROGRAM_SUCCESS = 0
PROGRAM_FAILURE = -1


def main() -> int:
    parser = argparse.ArgumentParser(description='Tune Reco parameters')
    parser.add_argument('reco_path', metavar='<reco_path>', type=str, help='Path to Reco executable.')
    parser.add_argument('configuration_path', metavar='<configuration_path>', type=str,
                        help='Default configuration file path.')
    parser.add_argument('until_frame', metavar='<until_frame>', type=int,
                        help='Run Reco until this frame (exclusive).')
    parser.add_argument('trials_step', metavar='<trials_step>', type=int,
                        help='How many new trials to do during this specific run.')
    parser.add_argument('--verbose', '-v', action='store_true', help="Produce verbose output.")
    parser.add_argument('--test2', '-t2', action='store_true',
                        help="Run in two-parameter test mode (tune learning rate and warp length threshold only).")
    parser.add_argument('--test1', '-t1', action='store_true',
                        help="Run in one-parameter test mode (tune learning rate and warp length threshold only).")
    args = parser.parse_args()

    if args.test1 and args.test2:
        raise ValueError("--test2 (-t2) and --test1 (-t1) arguments cannot be used at once.")

    mode = Mode.NORMAL
    if args.test1:
        mode = Mode.TEST_1_PARAMETER
    elif args.test2:
        mode = Mode.TEST_2_PARAMETERS

    tuner = Tuner(args.reco_path, args.configuration_path, args.until_frame, LossFunction.RMS_DISTANCE,
                  args.trials_step, args.verbose, mode)
    tuner.tune()

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
