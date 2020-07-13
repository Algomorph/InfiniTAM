#!/usr/bin/python3
import os
import sys
import argparse
from enum import Enum

import cv2
import cve
import pybgs
import numpy as np

PROGRAM_SUCCESS = 0
PROGRAM_ERROR = 1


class MaskLabel(Enum):
    FOREGROUND_LABEL = 255
    PERSISTENCE_LABEL = 180
    SHADOW_LABEL = 80
    BACKGROUND_LABEL = 0


class ConnectedComponentThreshold(Enum):
    HIDDEN = 1200
    BBOX_THRESH = 6500
    TRACK_DIST_THRESH = 60.


def main() -> None:
    parser = argparse.ArgumentParser("Generate foreground masks from given color frames using the IMBS color"
                                     " subtractor (Bloisi, Domenico, and Luca Iocchi. \"Independent multimodal"
                                     " background subtraction.\" In CompIMAGE, pp. 39-44. 2012.)")
    parser.add_argument("--frames_directory", "-f", type=str, default="/media/algomorph/Data/Reconstruction/real_data/snoopy/frames",
                        help="path to the frames folder. Color frames should have the following format:\n"
                             "<frames_directory>/color_XXXXXX.png\n"
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number.")
    parser.add_argument("--output_directory", "-o", type=str, default="/media/algomorph/Data/Reconstruction/real_data/snoopy/imbs_masks",
                        help="path to the output folder. Masks will be saved in the following format:\n"
                             "<output_directory>/imbs_mask_XXXXXX.png\n"
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number.")

    # ====================IMBS parameters========================== #
    parser.add_argument("--fps", type=float, default=30.0)

    parser.add_argument("--fg_threshold", type=int, default=15)
    parser.add_argument("--association_threshold", type=int, default=5)

    parser.add_argument("-sis", "--sampling_interval_start_frame", type=int, default=-1)
    parser.add_argument("-sie", "--sampling_interval_end_frame", type=int, default=-1)

    parser.add_argument("--sampling_interval", type=float, default=300.0,
                        help="Duration, in ms, of the sampling interval between frames"
                             " used to build initial background model")
    parser.add_argument("--num_samples", type=int, default=30)

    parser.add_argument("--min_bin_height", type=int, default=2)

    # *** shadow ***
    parser.add_argument("--alpha", type=float, default=0.65,
                        help="Lower bound on the value ratio between image & " +
                             "model for the pixel to be labeled shadow")
    parser.add_argument("--beta", type=float, default=1.15,
                        help="Upper bound on the value ratio between image & " +
                             "model for the pixel to be labeled shadow")
    parser.add_argument("--tau_s", type=float, default=60.,
                        help="Upper bound on the saturation difference between image & model "
                             "for the pixel to be labeled as shadow")
    parser.add_argument("--tau_h", type=float, default=40.,
                        help="Upper bound on the hue difference between image & model "
                             "for the pixel to be labeled as shadow")
    # ***************
    parser.add_argument("--min_area", type=float, default=1.15,
                        help="")

    parser.add_argument("--persistence_period", type=float, default=300.0,
                        help="Duration of the persistence period in ms")
    parser.add_argument("-m", "--use_morphological_filtering", help="Use morphological filtering (open, close) on "
                                                                    "the result.", default=False,
                        action='store_true')
    # ============================================================= #

    args = parser.parse_args()
    if not os.path.exists(args.output_directory):
        os.makedirs(args.output_directory)

    use_pybgs = False

    if use_pybgs:
        subtractor = pybgs.SuBSENSE()
    else:
        subtractor = cve.BackgroundSubtractorIMBS(args.fps, args.fg_threshold, args.association_threshold,
                                                  args.sampling_interval, args.min_bin_height, args.num_samples,
                                                  args.alpha, args.beta, args.tau_s, args.tau_h,
                                                  args.min_area, args.persistence_period,
                                                  args.use_morphological_filtering, False)

    folder_contents = os.listdir(args.frames_directory)
    color_frame_filenames = [filename for filename in folder_contents if filename.startswith('color_')]
    color_frame_filenames.sort()

    prev_frame_centroid = None

    for color_frame_filename in color_frame_filenames:
        no_ext_name = os.path.splitext(color_frame_filename)[0]
        frame_postfix = no_ext_name[-6:]
        color_frame_path = os.path.join(args.frames_directory, color_frame_filename)
        mask_image_path = os.path.join(args.output_directory, "imbs_mask_" + frame_postfix + ".png")
        print("Generating background mask for '{:s}' and saving it to '{:s}'".format(color_frame_path, mask_image_path))
        color_frame = cv2.imread(color_frame_path, cv2.IMREAD_UNCHANGED)
        mask = subtractor.apply(color_frame)

        if use_pybgs:
            cv2.imwrite(mask_image_path, mask)
        else:
            bin_mask = mask.copy()
            bin_mask[bin_mask < MaskLabel.PERSISTENCE_LABEL.value] = 0
            bin_mask[bin_mask > 0] = 1

            labels, stats, centroids = cv2.connectedComponentsWithStats(bin_mask, ltype=cv2.CV_16U)[1:4]
            if len(stats) > 1:
                # initially, just grab the biggest connected component
                ix_of_tracked_component = np.argmax(stats[1:, 4]) + 1
                largest_centroid = centroids[ix_of_tracked_component].copy()
                tracking_ok = True

                if prev_frame_centroid is not None:
                    a = prev_frame_centroid
                    b = largest_centroid
                    dist = np.linalg.norm(a - b)
                    # check to make sure we're not too far from the previously-detected blob
                    if dist > 50:
                        dists = np.linalg.norm(centroids - a, axis=1)
                        ix_of_tracked_component = np.argmin(dists)
                        if dists[ix_of_tracked_component] > ConnectedComponentThreshold.TRACK_DIST_THRESH.value:
                            tracking_ok = False
                        largest_centroid = centroids[ix_of_tracked_component].copy()

                tracked_px_count = stats[ix_of_tracked_component, 4]
                tracked_object_stats = stats[ix_of_tracked_component]
                contour_found = tracked_px_count > ConnectedComponentThreshold.HIDDEN.value and tracking_ok

                if contour_found:
                    bin_mask[labels != ix_of_tracked_component] = 0
                    prev_frame_centroid = largest_centroid
                else:
                    prev_frame_centroid = None

            cv2.imwrite(mask_image_path, bin_mask * 255)

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
