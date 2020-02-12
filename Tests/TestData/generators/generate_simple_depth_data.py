#!/usr/bin/python3
import sys

from matplotlib import cm
import numpy as np
import cv2


class Camera:
    def __init__(self):
        # camera intrinsics
        self.fx = 517
        self.fy = 517
        self.cx = 320
        self.cy = 240

        # image resolution
        self.im_width = 640
        self.im_height = 480

        # frustum clipping (in mm)
        self.clip_near = 200
        self.clip_far = 3000


def generate_stripes():
    cam = Camera()

    stripe_height = 32
    vertical_margin_size = (cam.im_height - stripe_height) // 2
    stripes_depth = np.zeros((cam.im_height, cam.im_width), dtype=np.uint16)

    stripe_count = (cam.im_width - 20) // 10

    distance_between_stripes = 40
    first_stripe_at_depth = cam.clip_near + distance_between_stripes
    for i_stripe in range(stripe_count):
        stripes_depth[vertical_margin_size:-vertical_margin_size, (10 + 10 * i_stripe): (10 + 10 * (i_stripe + 1))] = \
            first_stripe_at_depth + distance_between_stripes * i_stripe

    cv2.imwrite("stripes_depth.png", stripes_depth)

    progression = (stripes_depth.astype(np.float64) - cam.clip_near)
    progression[progression < 0] = 0
    progression = progression / progression.max()
    stripe_index = (progression * 62).astype(np.int64)

    # For grayscale stripes:
    # stripes_color = np.zeros((im_height,im_width,3),dtype=np.uint8)

    # for i_channel in range(3):
    # 	stripes_color[:,:,i_channel] = progression.astype(np.uint8)

    stripe_colors = (np.fliplr(
        np.array([cm.cividis(value)[0:3] for value in np.arange(0, 1.0, 1.0 / stripe_count)])) * 255).astype(np.uint8)

    # For stripes with consecutively close colors:
    # stripes_color = (cm.cividis(progression)[:,:,0:3] * 255).astype(np.uint8)

    # For stripes with highly-varying colors:
    new_stripe_colors = np.zeros((stripe_colors.shape[0] + 1, stripe_colors.shape[1]), dtype=np.uint8)

    for i_color in range(stripe_count):
        new_stripe_colors[i_color + 1] = stripe_colors[i_color // 2] if i_color % 2 == 0 else stripe_colors[
            stripe_count // 2 + i_color // 2]

    stripes_color = new_stripe_colors[stripe_index]

    cv2.imwrite("stripes_color.png", stripes_color)

    # u is the horizontal image coordinate
    # _c denotes "in camera space"
    # coordinates are expressed in mm

    def z_c(u):
        return first_stripe_at_depth + distance_between_stripes * ((u - 10) // 10)

    def x_c(u):
        return (z_c(u) * (u - cam.cx)) / cam.fx


def image_parallel_distance_at_depth(depth, pixel_distance, cam=Camera(), horizontal=True):
    """

    :param depth:
    :type depth: Number
    :param cam:
    :type cam: Camera
    :param pixel_distance:
    :type pixel_distance: Number
    :param horizontal:
    :type horizontal: bool
    :return:
    """
    focal_length = cam.fx if horizontal else cam.fy
    return depth * pixel_distance / focal_length


def draw_centered_square_depth(depth, square_side, cam):
    start_x = (cam.im_width - square_side) // 2
    end_x = start_x + square_side
    start_y = (cam.im_height - square_side) // 2
    end_y = start_y + square_side
    square_depth_image = np.zeros((cam.im_height, cam.im_width), dtype=np.uint16)
    square_depth_image[start_y:end_y, start_x:end_x] = depth
    return square_depth_image


def depth_to_color_cividis(depth_image, cam):
    """
    :param depth_image:
    :type depth_image: np.ndarray
    :param cam:
    :type cam: Camera
    :return:
    :rtype: np.ndarray
    """
    clip_range = cam.clip_far - cam.clip_near
    depth_ratio = (depth_image.astype(np.float32) - cam.clip_near) / clip_range
    image = (cm.cividis(depth_ratio)[:, :, 0:3] * 255).astype(np.uint8)
    image[depth_image == 0] = (0, 0, 0)
    return np.flip(image, axis=2)


def generate_squares():
    cam = Camera()
    square_side = 100

    depth_1 = 2000
    depth_2 = 2080

    square1_depth = draw_centered_square_depth(depth_1, square_side, cam)
    square2_depth = draw_centered_square_depth(depth_2, square_side, cam)

    square1_color = depth_to_color_cividis(square1_depth, cam)
    square2_color = depth_to_color_cividis(square2_depth, cam)

    cv2.imwrite("square1_depth.png", square1_depth)
    cv2.imwrite("square2_depth.png", square2_depth)
    cv2.imwrite("square1_color.png", square1_color)
    cv2.imwrite("square2_color.png", square2_color)


def main():
    # generate_stripes()

    # generate_squares()
    print(image_parallel_distance_at_depth(2.00, 100) / 0.004)
    print(image_parallel_distance_at_depth(2.08, 100) / 0.004)

    return 0


if __name__ == "__main__":
    sys.exit(main())
