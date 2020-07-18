#!/usr/bin/python3

######################################################################
# Envokes background subtraction.
# Requires a built & installed CVE (https://github.com/Algomorph/cve)
######################################################################

import os
import sys
import argparse
from enum import Enum
from typing import List

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
    TRACK_DIST_THRESH = 40.


def get_paths_with_prefix(prefix: str, directory: str) -> List[str]:
    folder_contents = os.listdir(directory)
    paths = [os.path.join(directory, filename) for filename in folder_contents if filename.startswith(prefix)]
    paths.sort()
    return paths


def label_image(image: np.ndarray, label: str) -> None:
    width = image.shape[1]
    height = image.shape[0]
    cv2.putText(image, label, (width - 100, height - 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (255, 255, 255), 1)


def main() -> int:
    parser = argparse.ArgumentParser("Generate foreground masks from given color frames using a background subtraction algorithm.")

    parser.add_argument("--frames_directory", "-f", type=str, default="/mnt/Data/Reconstruction/real_data/snoopy/frames",
                        help="path to the color frames folder. Color frames should have the following format:\n"
                             "<frames_directory>/color_XXXXXX.png\n"
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number.")
    parser.add_argument("--background_frames_directory", "-bf", type=str,
                        default="",
                        help="path to extra frames to start the processing (many algorithms need to build a background model, so "
                             "a separate folder with only simulated or real background frames may be useful). No output is generated"
                             "for the background frames. Background frame file names should follow this convention:\n "
                             " <output>/color_bg_XXXXXX.png "
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number.")
    parser.add_argument("--depth_frames_directory", "-df", type=str, default="/mnt/Data/Reconstruction/real_data/snoopy/frames",
                        help="path to the depth frames folder. Depth frames should have the following format:\n"
                             "<frames_directory>/depth_XXXXXX.png\n"
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number. Depth frames should have a single 16-bit"
                             " integer channel where depth to the surface at each pixel, in mm, has been recorded.")
    parser.add_argument("--output", "-o", type=str, default="/mnt/Data/Reconstruction/real_data/snoopy/imbs_masks",
                        help="Unless '--output_as_video' option is used, path to the output folder with frame masks as images. "
                             "Otherwise, path to the output video file with masks."
                             "Masks images will be saved in "
                             "the following format:\n"
                             "<output>/imbs_mask_XXXXXX.png\n"
                             ", where 'XXXXXX' is a zero-padded 6-digit frame number.")

    parser.add_argument("--output_as_video", "-v", default=False,
                        action='store_true', help="Save output as video instead of frames.")

    parser.add_argument("--label_frames", "-lf", default=False, action='store_true',
                        help="Label each output frame image with its frame index")

    parser.add_argument("--clip_depth", "-cd", default=False, action='store_true',
                        help="Use depth images to clip the mask to a specific depth value range, as specified by the "
                             "--near_clip and --far_clip arguments.")
    parser.add_argument("--near_clip", "-nc", type=int, default=0,
                        help="Distance to the near clipping plane in whole mm (only used with --clip_depth).")
    parser.add_argument("--far_clip", "-fc", type=int, default=10000,
                        help="Distance to the far clipping plane in whole mm (only used with --clip_depth).")

    # ====================IMBS parameters========================== #
    parser.add_argument("--fps", type=float, default=30.0)

    parser.add_argument("--fg_threshold", type=int, default=15)
    parser.add_argument("--association_threshold", type=int, default=5)

    parser.add_argument("--sampling_interval", type=float, default=300.0,
                        help="Duration, in frames, of the sampling interval used to build "
                             "initial background model. Overridden in case the '--background_frames_directory' "
                             "setting is used and the directory provides proper background frame files, "
                             "in which case the interval is calculated from the number of frames and the '--fps' "
                             "setting.")
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
    parser.add_argument("--min_area", type=float, default=1.15, help="")

    parser.add_argument("--persistence_period", type=float, default=10000.0,
                        help="Duration of the persistence period in ms")
    parser.add_argument("-m", "--use_morphological_filter", default=False,
                        action='store_true', help="Use morphological filtering (open, close) on the result.")
    # ============================================================= #

    parser.add_argument("-pm", "--use_postprocessing_morphological_filter", default=False,
                        action='store_true', help="Use morphological filtering (open, close) on the result after postprocessing.")
    parser.add_argument("-pmks", "--postprocessing_morphological_kernel_size", default=5,
                        type=int, help="Size of kernel for postprocessing morphological filter.")

    args = parser.parse_args()
    if not os.path.exists(args.output):
        if args.output_as_video:
            if not os.path.exists(os.path.dirname(args.output)):
                os.makedirs(os.path.dirname(args.output))
        else:
            os.makedirs(args.output)

    background_color_frame_paths = []
    if len(args.background_frames_directory) > 0:
        background_color_frame_paths = get_paths_with_prefix('color_bg_', args.background_frames_directory)
        # args.sampling_interval = args.fps * 1000 * len(background_color_frame_paths)
        args.sampling_interval = len(background_color_frame_paths)

    color_frame_paths = get_paths_with_prefix('color_', args.frames_directory)
    depth_frame_paths = get_paths_with_prefix('depth_', args.depth_frames_directory)

    use_pybgs = False
    if use_pybgs:
        # TODO: add support for other BGMS models
        subtractor = pybgs.SuBSENSE()
    else:
        subtractor = cve.BackgroundSubtractorIMBS(args.fps, args.fg_threshold, args.association_threshold,
                                                  args.sampling_interval, args.min_bin_height, args.num_samples,
                                                  args.alpha, args.beta, args.tau_s, args.tau_h,
                                                  args.min_area, args.persistence_period,
                                                  args.use_morphological_filter, False)

    for frame_path in background_color_frame_paths:
        background_frame = cv2.imread(frame_path, cv2.IMREAD_UNCHANGED)
        print("Constructing background model from '{:s}'.".format(frame_path))
        subtractor.apply(background_frame)

    video_writer = None
    if args.output_as_video:
        color_frame = cv2.imread(color_frame_paths[0], cv2.IMREAD_UNCHANGED)
        # cv2.VideoWriter_fourcc('X', '2', '6', '4'), <-- for manually-compiled OpenCV w/ FFMPEG+x264
        video_writer = cv2.VideoWriter(args.output,
                                       cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                       args.fps, (color_frame.shape[1], color_frame.shape[0]), False)

    prev_frame_centroid = None
    i_frame = 0
    for frame_path in color_frame_paths:
        no_ext_filename = os.path.splitext(os.path.basename(frame_path))[0]
        frame_postfix = no_ext_filename[-6:]
        color_frame = cv2.imread(frame_path, cv2.IMREAD_UNCHANGED)
        mask = subtractor.apply(color_frame)
        print("Generating background mask for '{:s}'.".format(frame_path))

        if use_pybgs:
            output_image = mask
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
                # tracked_object_stats = stats[ix_of_tracked_component]
                contour_found = tracked_px_count > ConnectedComponentThreshold.HIDDEN.value and tracking_ok

                if contour_found:
                    bin_mask[labels != ix_of_tracked_component] = 0
                    prev_frame_centroid = largest_centroid
                else:
                    prev_frame_centroid = None

            output_image = bin_mask * 255

        if args.use_postprocessing_morphological_filter:
            kernel_open = np.ones((3, 3), np.uint8)
            kernel_close = np.ones((args.postprocessing_morphological_kernel_size, args.postprocessing_morphological_kernel_size), np.uint8)
            output_image = cv2.morphologyEx(output_image, cv2.MORPH_OPEN, kernel_open)
            output_image = cv2.morphologyEx(output_image, cv2.MORPH_CLOSE, kernel_close)

        if args.clip_depth:
            depth_path = depth_frame_paths[i_frame]
            depth_frame = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            # do not erase depth values missing due to occlusion
            depth_frame[depth_frame == 0] = args.near_clip
            output_image[depth_frame < args.near_clip] = 0
            output_image[depth_frame > args.far_clip] = 0

        if args.label_frames:
            label_image(output_image, frame_postfix)

        if args.output_as_video:
            video_writer.write(output_image)
        else:
            mask_image_path = os.path.join(args.output, "imbs_mask_" + frame_postfix + ".png")
            print("Saving resulting mask to '{:s}'".format(frame_path, mask_image_path))
            cv2.imwrite(mask_image_path, output_image)

        i_frame += 1

    if args.output_as_video:
        video_writer.release()

    return PROGRAM_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
