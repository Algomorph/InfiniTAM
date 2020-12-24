import sys
import os
import os.path
import argparse
import cv2
import numpy as np
import numpy.random as rnd


def make_labeling_consistent(image, previous_label_mapping=None):
    """
    # make labels consistently numbered from 0, i.e. 0, 1, 2, ... N, where N is the number of segments
    :param previous_label_mapping: if specified, the function will use this mapping of old values to new in addition to
    a new mapping for those values in the source image that aren't in this mapping yet
    :param image: input segmentation image
    :rtype: (numpy.ndarray, dict)
    :return: same segmentation with labels consistently numbered from 0 and up,
    the (complete) pixel mapping that was used
    """
    label_mapping = {}
    min_new_label = 0
    prev_target_values = []

    #
    if previous_label_mapping is not None and len(previous_label_mapping) > 0:
        label_mapping = previous_label_mapping
        prev_target_values = list(label_mapping.values())
        min_new_label = max(prev_target_values) + 1

    if len(image.shape) == 2:
        old_label_values = list(np.unique(image))
        old_values_not_in_mapping = []

        # filter the image values that aren't in the mapping yet
        for val in old_label_values:
            if val not in label_mapping:
                old_values_not_in_mapping.append(val)

        source_values = list(label_mapping.keys()) + old_values_not_in_mapping
        target_values = prev_target_values + list(range(min_new_label, min_new_label+len(old_values_not_in_mapping)))

        new_image = np.zeros_like(image)
        for (old_label, new_label) in zip(source_values, target_values):
            new_image[image == old_label] = new_label
            label_mapping[old_label] = new_label
    else:
        new_image = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        flat_image = image.reshape(new_image.size, -1)
        old_label_values = list(np.unique(flat_image, axis=0))
        old_values_not_in_mapping = []

        # filter the image values that aren't in the mapping yet
        for val in old_label_values:
            if tuple(val) not in label_mapping:
                old_values_not_in_mapping.append(val)

        source_values = list(label_mapping.keys()) + old_values_not_in_mapping
        target_values = prev_target_values + list(range(min_new_label, min_new_label + len(old_values_not_in_mapping)))

        for (old_label, new_label) in zip(source_values, target_values):
            new_image[(image == old_label).sum(axis=2) == 3] = new_label
            label_mapping[tuple(old_label)] = new_label

    return new_image, label_mapping


def thin(img):
    """
    Performs skeletonization on the image using morphology operators
    :param img: input image
    :return: skeletonized image
    """
    done = False
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    skel = np.zeros_like(img)
    while not done:
        eroded = cv2.erode(img, element)
        temp = img - cv2.dilate(eroded, element)
        cv2.bitwise_or(skel, temp, skel)
        img[:] = eroded[:]
        done = cv2.countNonZero(img) == 0
    return skel


def draw_contours(input_image, threshold1=50, threshold2=200, do_thinning=False):
    """
    Find and draw contours based on connected components in the image.
    Canny thresholding can be adjusted to make contours more/less sensitive. I.e. for segments drawn with values
    0,1,2,3 ... it makes sense to use thresholds of 0 and 1 respectively.
    NB: the output image is binary, i.e. only uses values 0 and 1
    :param threshold2: Canny higher threshold
    :param threshold1: Canny lower threshold
    :param input_image: the input image
    :param do_thinning: whether to thin/skeletonize the contours after they are produced
    :return:
    """
    cannied = cv2.Canny(input_image, threshold1, threshold2)
    contours = cv2.findContours(cannied, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    contour_image = np.zeros((input_image.shape[0], input_image.shape[1]), np.uint8)
    cv2.drawContours(contour_image, contours, -1, 1)
    if do_thinning:
        return thin(contour_image)
    else:
        return contour_image


def make_dir_if_missing(path):
    if not os.path.isdir(path):
        if os.path.exists(path):
            raise ValueError("{:s} exists, but is not a directory path! Aborting.".format(path))
        os.makedirs(path)
