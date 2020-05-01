#!/usr/bin/python3
import sys

from Apps.visualizer.visualizerapp import VisualizerApp

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


def main():
    visualizer = VisualizerApp()
    visualizer.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
