from enum import Enum
import cv2
import numpy as np


class FrameImageType(Enum):
    COLOR = 1
    DEPTH = 2
    MASK = 3


def generate_frame_image_path(index, frame_image_type):
    """
    :type index int
    :type frame_image_type FrameImageType
    :rtype str
    """
    base_path = "/mnt/Data/Reconstruction/real_data/snoopy/frames"
    filename_prefix_by_frame_image_type = {
        FrameImageType.COLOR: "color",
        FrameImageType.DEPTH: "depth",
        FrameImageType.MASK: "omask"
    }
    return "{:s}/{:s}_{:06d}.png".format(base_path, filename_prefix_by_frame_image_type[frame_image_type], index)

def load_frame_numpy_raw_image(index, frame_image_type):
    return cv2.imread(generate_frame_image_path(index, frame_image_type),cv2.IMREAD_UNCHANGED)

def load_mask_numpy_image(index):
    return load_frame_numpy_raw_image(index, FrameImageType.MASK).astype(bool)

def load_depth_numpy_image(index, conversion_factor = 0.001):
    return load_frame_numpy_raw_image(index, FrameImageType.DEPTH).astype(np.float32) * conversion_factor


def load_color_numpy_image(index):
    return load_frame_numpy_raw_image(index, FrameImageType.COLOR)
