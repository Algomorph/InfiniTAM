import math
import os

import vtk
import gzip
import numpy as np
from vtk.util import numpy_support as ns
from typing import List

from Apps.python_shared import trajectory_loading
from Apps.visualizer.geometric_conversions import convert_block_to_metric
from Apps.visualizer.utilities import get_output_frame_count, get_frame_output_path


class FrameBlockAllocationRayData:
    def __init__(self, live_surface_points, canonical_surface_points, march_segment_endpoints):
        self.live_surface_points = live_surface_points
        self.canonical_surface_points = canonical_surface_points
        self.march_segment_endpoints = march_segment_endpoints


def read_block_allocation_ray_data(inverse_camera_matrices,
                                   output_path: str,
                                   initial_frame_index: int,
                                   frame_bound=-1) -> List[FrameBlockAllocationRayData]:
    frame_ray_datasets = [None]
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
            bool_layer_shape = tuple(np.frombuffer(file.read(size=2 * np.dtype(np.int32).itemsize), dtype=np.int32))
            channel_count = int(np.frombuffer(file.read(size=1 * np.dtype(np.int32).itemsize), dtype=np.int32)[0])
            if channel_count != 1:
                raise ValueError("Expected a a channel_count of 1, got {:d}".format(channel_count))
            layer_element_count = bool_layer_shape[0] * bool_layer_shape[1]
            layer = np.frombuffer(file.read(layer_element_count * np.dtype(np.bool).itemsize),
                                  dtype=np.bool)
            layers.append(layer)

        num_float_layers = int(np.frombuffer(file.read(size=np.dtype(np.int32).itemsize), dtype=np.int32)[0])
        for i_layer in range(0, num_float_layers):
            float_layer_shape = tuple(np.frombuffer(file.read(size=3 * np.dtype(np.int32).itemsize), dtype=np.int32))
            channel_count = float_layer_shape[2]
            value_count = int(float_layer_shape[0] * float_layer_shape[1] * channel_count)
            layer = np.frombuffer(file.read(value_count * np.dtype(np.float32).itemsize),
                                  dtype=np.float32).reshape(-1, channel_count)
            layers.append(layer)

        point_mask1 = layers[0]
        point_mask2 = layers[1]
        segment_mask = np.logical_or(layers[0], layers[1])

        inverse_camera_matrix_current = inverse_camera_matrices[i_frame - initial_frame_index]
        inverse_camera_matrix_prev = inverse_camera_matrices[i_frame - initial_frame_index - 1]
        # inverse_camera_matrix_next = inverse_camera_matrices[i_frame - initial_frame_index + 1]
        camera_matrix_current = np.linalg.inv(inverse_camera_matrix_current)
        identity_matrix = np.identity(4, dtype=np.float32)
        camera_matrix_canonical = identity_matrix

        live_based_point_cloud = layers[2][point_mask1]
        one_col = np.ones((live_based_point_cloud.shape[0], 1), dtype=np.float32)
        live_based_point_cloud = inverse_camera_matrix_current.dot(np.hstack((live_based_point_cloud, one_col)).T).T[:, 0:3]

        canonical_based_point_cloud = layers[3][point_mask2]
        one_col = np.ones((canonical_based_point_cloud.shape[0], 1), dtype=np.float32)
        canonical_based_point_cloud = camera_matrix_canonical.dot(np.hstack((canonical_based_point_cloud, one_col)).T).T[:, 0:3]

        # index_cols = [2, 0, 1] [:, index_cols]
        march_segment_endpoints = np.hstack((convert_block_to_metric(layers[4][segment_mask]),
                                             convert_block_to_metric(layers[5][segment_mask])))

        frame_ray_datasets.append(FrameBlockAllocationRayData(live_based_point_cloud, canonical_based_point_cloud,
                                                              march_segment_endpoints))

    return frame_ray_datasets


class AllocationRays:

    def __init__(self, renderer, output_path, initial_frame_index, inverted_camera_matrices):
        colors = vtk.vtkNamedColors()
        self.inverted_camera_matrices = inverted_camera_matrices
        self.output_path = output_path
        self.initial_frame_index = initial_frame_index
        self.frame_ray_datasets = read_block_allocation_ray_data(inverted_camera_matrices, output_path,
                                                                 initial_frame_index)

        self.current_frame_index = initial_frame_index

        self.point_cloud1_data = vtk.vtkPolyData()
        self.point_cloud1_glyph_filter = vtk.vtkVertexGlyphFilter()
        self.point_cloud1_glyph_filter.SetInputData(self.point_cloud1_data)
        self.point_cloud1_mapper = vtk.vtkPolyDataMapper()
        self.point_cloud1_mapper.SetInputConnection(self.point_cloud1_glyph_filter.GetOutputPort())
        self.point_cloud1_actor = vtk.vtkActor()
        self.point_cloud1_actor.GetProperty().SetColor(colors.GetColor3d("Orange"))
        self.point_cloud1_actor.GetProperty().SetPointSize(2)
        self.point_cloud1_actor.SetMapper(self.point_cloud1_mapper)

        self.point_cloud2_data = vtk.vtkPolyData()
        self.point_cloud2_glyph_filter = vtk.vtkVertexGlyphFilter()
        self.point_cloud2_glyph_filter.SetInputData(self.point_cloud2_data)
        self.point_cloud2_mapper = vtk.vtkPolyDataMapper()
        self.point_cloud2_mapper.SetInputConnection(self.point_cloud2_glyph_filter.GetOutputPort())
        self.point_cloud2_actor = vtk.vtkActor()
        self.point_cloud2_actor.GetProperty().SetColor(colors.GetColor3d("Peacock"))
        self.point_cloud2_actor.GetProperty().SetPointSize(2)
        self.point_cloud2_actor.SetMapper(self.point_cloud2_mapper)

        self.segment_data = vtk.vtkPolyData()
        self.segment_mapper = vtk.vtkPolyDataMapper()
        self.segment_mapper.SetInputData(self.segment_data)
        self.segment_actor = vtk.vtkActor()
        self.segment_actor.SetMapper(self.segment_mapper)

        # renderer.AddActor(self.segment_actor)
        renderer.AddActor(self.point_cloud1_actor)
        renderer.AddActor(self.point_cloud2_actor)
        self.renderer = renderer

    def set_frame(self, frame_index):
        print(frame_index, len(self.frame_ray_datasets), self.initial_frame_index)
        if self.initial_frame_index < frame_index != self.current_frame_index:
            dataset = self.frame_ray_datasets[frame_index - self.initial_frame_index]

            point_cloud1_points = vtk.vtkPoints()
            for x, y, z in dataset.live_surface_points:
                point_cloud1_points.InsertNextPoint(x, -y, z)

            self.point_cloud1_data.SetPoints(point_cloud1_points)
            self.point_cloud1_glyph_filter.SetInputData(self.point_cloud1_data)
            self.point_cloud1_glyph_filter.Update()
            self.point_cloud1_mapper.Modified()

            point_cloud2_points = vtk.vtkPoints()
            for x, y, z in dataset.canonical_surface_points:
                point_cloud2_points.InsertNextPoint(x, -y, z)

            self.point_cloud2_data.SetPoints(point_cloud2_points)
            self.point_cloud2_glyph_filter.SetInputData(self.point_cloud2_data)
            self.point_cloud2_glyph_filter.Update()
            self.point_cloud2_mapper.Modified()

            segment_points = vtk.vtkPoints()
            lines = vtk.vtkCellArray()

            point_id = 0
            for x0, y0, z0, x1, y1, z1 in dataset.march_segment_endpoints:
                # print(x0, y0, z0, "|", x1, y1, z1)
                # points.InsertNextPoint(x0, -y0, z0)
                # points.InsertNextPoint(x0 + 0.01, -y0 + 0.01, z0 + 0.01)
                segment_points.InsertNextPoint(x1, -y1, z1)
                segment_points.InsertNextPoint(x1 + 0.01, -y1 + 0.01, z1 + 0.01)
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, point_id)
                line.GetPointIds().SetId(1, point_id + 1)
                lines.InsertNextCell(line)
                point_id += 2

            self.segment_data.SetPoints(segment_points)
            self.segment_data.SetLines(lines)

            self.segment_mapper.SetInputData(self.segment_data)
            self.segment_mapper.Modified()

            self.current_frame_index = frame_index


# test
if __name__ == "__main__":
    inverse_camera_matrices = trajectory_loading.load_inverse_matrices(
        "/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording")
    read_block_allocation_ray_data(inverse_camera_matrices,
                                   "/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording", 16, 18)
