#!/usr/bin/python3
import sys
import argparse

from Apps.visualizer.visualizerapp import VisualizerApp

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


def main():
    parser = argparse.ArgumentParser("App for visualizing block allocation and generated mesh alignment.")
    parser.add_argument("--output", "-o", type=str, help="Path to InfiniTAM output_folder",
                        default="/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording")
    args = parser.parse_args()2)
    visualizer = VisualizerApp(args.output)
    visualizer.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
