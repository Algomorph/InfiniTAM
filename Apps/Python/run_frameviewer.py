#!/usr/bin/python3
import sys
import argparse
from Apps.Python.frameviewer.frameviewer import FrameViewerApp
from Apps.Python.shared import screen_management

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


def main():
    parser = argparse.ArgumentParser("App for visualizing RGB-D frame data.")
    parser.add_argument("--output", "-o", type=str, help="Path to InfiniTAM output_folder",
                        default="/mnt/Data/Reconstruction/experiment_output/2020-07-03/recording_snoopy")
    args = parser.parse_args()
    print("Reading data from ", args.output)

    app = FrameViewerApp(args.output, 16)
    app.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
