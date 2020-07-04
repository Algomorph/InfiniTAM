import gzip
import os

import numpy as np
from typing import List

from Apps.Python.shared import trajectory_loading
from Apps.Python.visualizer.geometric_conversions import convert_block_to_metric
from Apps.Python.visualizer.utilities import get_output_frame_count, get_frame_output_path


class FrameBlockData:
    def __init__(self, block_coordinates, metric_block_coordinates):
        self.block_coordinates = block_coordinates
        self.metric_block_coordinates = metric_block_coordinates

    def make_label_text(self, i_block: int) -> str:
        block_coordinate = self.block_coordinates[i_block]
        label = "({:d}, {:d}, {:d})".format(block_coordinate[0],
                                            block_coordinate[1],
                                            block_coordinate[2])
        return label


def read_canonical_block_allocation_data(data_path: str, i_frame: int) -> List[FrameBlockData]:
    file = gzip.open(data_path, "rb")

    block_sets = []
    metric_sets = []
    while file.readable():
        buffer = file.read(size=8)
        if not buffer:
            break
        count = np.frombuffer(buffer, dtype=np.uint32)[0]
        print("Reading allocation data for frame:", i_frame, "Block count:", count, "...")
        block_set = np.resize(np.frombuffer(file.read(size=6 * count), dtype=np.int16), (count, 3))
        metric_set = convert_block_to_metric(block_set)
        block_sets.append(FrameBlockData(block_set, metric_set))
        i_frame += 1
    return block_sets


class FrameBlockAllocationRayData:
    def __init__(self, live_surface_points, canonical_surface_points, march_segment_endpoints):
        self.live_surface_points = live_surface_points
        self.canonical_surface_points = canonical_surface_points
        self.march_segment_endpoints = march_segment_endpoints


class FramePixelBlockData(FrameBlockData):
    def __init__(self, pixel_coordinates, block_coordinates, metric_block_coordinates):
        super().__init__(block_coordinates, metric_block_coordinates)
        self.pixel_coordinates = pixel_coordinates

    def make_label_text(self, i_block: int) -> str:
        label_block_coordinate = super().make_label_text(i_block)
        pixel_coordinate = self.pixel_coordinates[i_block]
        label_pixel = "\n px: ({:d}, {:d})".format(pixel_coordinate[0], pixel_coordinate[1])
        return label_block_coordinate + label_pixel


def read_2d_image(file, dtype=np.bool) -> np.ndarray:
    image_shape = tuple(np.frombuffer(file.read(size=2 * np.dtype(np.int32).itemsize), dtype=np.int32))
    channel_count = int(np.frombuffer(file.read(size=1 * np.dtype(np.int32).itemsize), dtype=np.int32)[0])
    if channel_count != 1:
        raise ValueError("Expected a a channel_count of 1, got {:d}".format(channel_count))
    value_count = image_shape[0] * image_shape[1]
    image_shape = (image_shape[1], image_shape[0])
    return np.frombuffer(file.read(value_count * np.dtype(dtype).itemsize), dtype=dtype).reshape(image_shape)


def read_pixel_array_from_2d_image(file, dtype=np.bool) -> np.ndarray:
    image = read_2d_image(file, dtype)
    return image.flatten()


def read_4d_image(file, dim4=3, dtype=np.int16) -> np.ndarray:
    image_shape = tuple(list(np.frombuffer(file.read(size=3 * np.dtype(np.int32).itemsize), dtype=np.int32)) + [dim4])
    value_count = int(image_shape[0] * image_shape[1] * image_shape[2] * dim4)
    image_shape = (image_shape[1], image_shape[0], image_shape[2], dim4)
    return np.frombuffer(file.read(value_count * np.dtype(dtype).itemsize), dtype=dtype).reshape(image_shape)


def read_3d_image(file, dtype=np.float32) -> np.ndarray:
    image_shape = tuple(np.frombuffer(file.read(size=3 * np.dtype(np.int32).itemsize), dtype=np.int32))
    channel_count = image_shape[2]
    value_count = int(image_shape[0] * image_shape[1] * channel_count)
    image_shape = (image_shape[1], image_shape[0], channel_count)
    return np.frombuffer(file.read(value_count * np.dtype(dtype).itemsize), dtype=dtype).reshape(image_shape)


def read_pixel_array_from_3d_image(file, dtype=np.float32) -> np.ndarray:
    image = read_3d_image(file, dtype)
    channel_count = image.shape[2]
    return image.reshape(-1, channel_count)


def compile_ray_block_data(layers: List[np.ndarray], inverse_camera_matrix: np.ndarray) -> FrameBlockAllocationRayData:
    point_mask1 = layers[0]
    point_mask2 = layers[1]
    segment_mask = np.logical_or(layers[0], layers[1])

    live_based_point_cloud = layers[2][point_mask1]
    one_col = np.ones((live_based_point_cloud.shape[0], 1), dtype=np.float32)
    live_based_point_cloud = inverse_camera_matrix.dot(np.hstack((live_based_point_cloud, one_col)).T).T[:, 0:3]

    canonical_based_point_cloud = layers[3][point_mask2]
    one_col = np.ones((canonical_based_point_cloud.shape[0], 1), dtype=np.float32)
    canonical_based_point_cloud = inverse_camera_matrix.dot(np.hstack((canonical_based_point_cloud, one_col)).T).T[:, 0:3]

    # index_cols = [2, 0, 1] [:, index_cols]
    march_segment_endpoints = np.hstack((convert_block_to_metric(layers[4][segment_mask]),
                                         convert_block_to_metric(layers[5][segment_mask])))
    return FrameBlockAllocationRayData(live_based_point_cloud, canonical_based_point_cloud, march_segment_endpoints)


def compile_pixel_block_data(pixel_block_allocations_image: np.ndarray, pixel_block_allocation_count_image: np.ndarray) -> FramePixelBlockData:
    pixel_block_allocations = pixel_block_allocations_image.reshape(-1, pixel_block_allocations_image.shape[2],
                                                                    pixel_block_allocations_image.shape[3])
    pixel_block_allocation_counts = pixel_block_allocation_count_image.reshape(-1, 1)
    used_pixel_block_allocation_mask = np.tile(np.arange(pixel_block_allocations.shape[1]), (pixel_block_allocations.shape[0], 1))
    pixel_grid = np.zeros((pixel_block_allocations_image.shape[0], pixel_block_allocations_image.shape[1], 2), dtype=np.int16)
    pixel_grid[:, :, 1] = np.tile(np.arange(pixel_block_allocations_image.shape[0], dtype=np.int16).reshape(-1, 1),
                                  (1, pixel_block_allocations_image.shape[1]))
    pixel_grid[:, :, 0] = np.tile(np.arange(pixel_block_allocations_image.shape[1], dtype=np.int16), (pixel_block_allocations_image.shape[0], 1))
    pixel_coordinates = pixel_grid.reshape(-1, 2)
    pixel_coordinates_tiled = np.tile(pixel_coordinates.reshape(pixel_coordinates.shape[0], 1, pixel_coordinates.shape[1]),
                                      (1, used_pixel_block_allocation_mask.shape[1], 1))
    pixel_index = used_pixel_block_allocation_mask < pixel_block_allocation_counts
    effective_pixel_coordinates = pixel_coordinates_tiled[pixel_index]
    allocated_blocks_by_effective_pixel = pixel_block_allocations[pixel_index]
    metric_block_coordinates = convert_block_to_metric(allocated_blocks_by_effective_pixel)
    return FramePixelBlockData(effective_pixel_coordinates, allocated_blocks_by_effective_pixel, metric_block_coordinates)


def read_live_block_allocation_data(inverse_camera_matrices: List[np.ndarray],
                                    output_path: str,
                                    initial_frame_index: int,
                                    frame_bound=-1) -> (List[FrameBlockAllocationRayData], List[FramePixelBlockData]):
    frame_ray_datasets = [FrameBlockAllocationRayData(np.array([]), np.array([]), np.array([]))]
    frame_pixel_block_datasets = [FramePixelBlockData(np.array([]), np.array([]), np.array([]))]
    if frame_bound != -1:
        total_frame_count = frame_bound
    else:
        total_frame_count = get_output_frame_count(output_path)

    for i_frame in range(initial_frame_index + 1, initial_frame_index + total_frame_count):
        print("Reading ray allocation data for frame:", i_frame)
        frame_folder = get_frame_output_path(output_path, i_frame)
        data_path = os.path.join(frame_folder, "voxel_block_hash_diagnostic_data.dat")
        file = gzip.open(data_path, "rb")

        layers = []

        num_bool_layers = int(np.frombuffer(file.read(size=np.dtype(np.int32).itemsize), dtype=np.int32)[0])
        for i_layer in range(0, num_bool_layers):
            layer = read_pixel_array_from_2d_image(file)
            layers.append(layer)

        num_float_layers = int(np.frombuffer(file.read(size=np.dtype(np.int32).itemsize), dtype=np.int32)[0])
        for i_layer in range(0, num_float_layers):
            layer = read_pixel_array_from_3d_image(file)
            layers.append(layer)

        inverse_camera_matrix = inverse_camera_matrices[i_frame - initial_frame_index]
        frame_ray_dataset = compile_ray_block_data(layers, inverse_camera_matrix)
        frame_ray_datasets.append(frame_ray_dataset)

        pixel_block_allocations = read_4d_image(file, 3, np.int16)
        pixel_block_allocation_counts = read_2d_image(file, np.int32)

        frame_pixel_block_dataset = compile_pixel_block_data(pixel_block_allocations, pixel_block_allocation_counts)
        frame_pixel_block_datasets.append(frame_pixel_block_dataset)

    return frame_ray_datasets, frame_pixel_block_datasets


# test
if __name__ == "__main__":
    inverse_camera_matrices = trajectory_loading.load_inverse_matrices(
        "/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording")
    read_live_block_allocation_data(inverse_camera_matrices,
                                    "/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording", 16, 18)
