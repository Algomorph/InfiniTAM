#!/usr/bin/python3
import sys
import argparse

# Note: the current intent is for this script to be run (perhaps from within a python IDE) with the repository root in the PYTHONPATH. If you try
# to run the scripts from its own directory it won't work. Try running from repository root instead.
from Apps.visualizer.visualizerapp import VisualizerApp

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


def main():
    parser = argparse.ArgumentParser("App for visualizing block allocation and generated mesh alignment.")
    parser.add_argument("--output", "-o", type=str, help="Path to InfiniTAM output_folder",
                        default="/mnt/Data/Reconstruction/experiment_output/2020-06-15/recording")
    args = parser.parse_args()
    print("Reading data from ", args.output)
    visualizer = VisualizerApp(args.output)
    visualizer.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
