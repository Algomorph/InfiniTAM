import numpy as np
import math
from typing import Mapping, Tuple


class Segment:
    def __init__(self, start: np.ndarray, end: np.ndarray):
        self.start = np.array(start)
        self.end = np.array(end)
        self.direction = self.end - self.start
        self.inverse_direction = 1 / self.direction
        self.sign = self.inverse_direction < 0.0

    def length(self):
        return np.linalg.norm(self.direction)

    def __str__(self):
        return "start: " + str(self.start) + ", end: " + str(self.end)

    def __repr__(self):
        return "start: " + repr(self.start) + ", end: " + repr(self.end)


class GridAlignedBox:
    def __init__(self, min_point: np.ndarray, max_point: np.ndarray):
        self.min_point = min_point
        self.max_point = max_point
        self.bounds = (min_point, max_point)

    def __str__(self):
        return "min_point: " + str(self.min_point) + ", max_point: " + str(self.max_point)

    def __repr__(self):
        return "min_point: " + repr(self.min_point) + ", max_point: " + repr(self.max_point)


def get_voxel_block_extent_metric(block_coordinates: np.ndarray, voxel_size=0.004, voxel_block_size=8):
    return GridAlignedBox(np.array(block_coordinates, dtype=np.float) * voxel_block_size * voxel_size - voxel_size / 2,
                          (np.array(block_coordinates, dtype=np.float) + np.array([1, 1, 1])) * voxel_size * voxel_block_size - voxel_size / 2)


def get_voxel_extent_metric(voxel_coordinates: np.ndarray, voxel_size=0.004):
    return GridAlignedBox(np.array(voxel_coordinates, dtype=np.float) * voxel_size - voxel_size / 2,
                          (np.array(voxel_coordinates, dtype=np.float) + np.array([1, 1, 1])) * voxel_size - voxel_size / 2)


def convert_point_blocks(point: np.ndarray, voxel_size=0.004, voxel_block_size=8):
    return point / (voxel_size * voxel_block_size) + 1 / (2 * voxel_block_size)


def get_voxel_block_extent_blocks(block_coordinates: np.ndarray):
    return GridAlignedBox(np.array(block_coordinates, dtype=np.float), (np.array(block_coordinates, dtype=np.float) + np.array([1, 1, 1])))


def segment_intersects_grid_aligned_box(segment: Segment, box: GridAlignedBox):
    bounds = box.bounds
    sign = segment.sign
    start = segment.start
    inverse_direction = segment.inverse_direction

    t_min = (bounds[sign[0]][0] - start[0]) * inverse_direction[0]
    t_max = (bounds[1 - sign[0]][0] - start[0]) * inverse_direction[0]
    t_y_min = (bounds[sign[1]][1] - start[1]) * inverse_direction[1]
    t_y_max = (bounds[1 - sign[1]][1] - start[1]) * inverse_direction[1]

    if (t_min > t_y_max) or (t_y_min > t_max):
        return False

    if t_y_min > t_min:
        t_min = t_y_min
    if t_y_max < t_max:
        t_max = t_y_max

    t_z_min = (bounds[segment.sign[2]][2] - start[2]) * segment.inverse_direction[2]
    t_z_max = (bounds[1 - segment.sign[2]][2] - start[2]) * segment.inverse_direction[2]

    if (t_min > t_z_max) or (t_z_min > t_max):
        return False

    if t_z_min > t_min:
        t_min = t_z_min

    if t_z_max < t_max:
        t_max = t_z_max
    return not (t_max < 0.0 or t_min > 1.0)


def generate_march_points(segment):
    step_count = math.ceil(2.0 * segment.length())
    stride_vector = segment.direction / (step_count - 1)

    checked_position = segment.start
    march_points = []
    for i_step in range(0, step_count):
        march_points.append(checked_position.copy())
        checked_position += stride_vector

    return march_points
