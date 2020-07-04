import os

import vtk


class AllocationRaySourcePointCloud:
    def __init__(self, color_name):
        colors = vtk.vtkNamedColors()
        self.__point_cloud_data = vtk.vtkPolyData()
        self.__point_cloud_glyph_filter = vtk.vtkVertexGlyphFilter()
        self.__point_cloud_glyph_filter.SetInputData(self.__point_cloud_data)
        self.__point_cloud_mapper = vtk.vtkPolyDataMapper()
        self.__point_cloud_mapper.SetInputConnection(self.__point_cloud_glyph_filter.GetOutputPort())
        self.__point_cloud_actor = vtk.vtkActor()
        self.__point_cloud_actor.GetProperty().SetColor(colors.GetColor3d(color_name))
        self.__point_cloud_actor.GetProperty().SetPointSize(2)
        self.__point_cloud_actor.SetMapper(self.__point_cloud_mapper)

    def update_points(self, point_array):
        point_cloud_points = vtk.vtkPoints()
        for x, y, z in point_array:
            point_cloud_points.InsertNextPoint(x, -y, z)

        self.__point_cloud_data.SetPoints(point_cloud_points)
        self.__point_cloud_glyph_filter.SetInputData(self.__point_cloud_data)
        self.__point_cloud_glyph_filter.Update()
        self.__point_cloud_mapper.Modified()

    def get_actor(self):
        return self.__point_cloud_actor

    def toggle_visibility(self):
        self.__point_cloud_actor.SetVisibility(not self.__point_cloud_actor.GetVisibility())


class AllocationRays:
    def __init__(self, renderer, output_path, initial_frame_index, inverted_camera_matrices, frame_ray_datasets):
        colors = vtk.vtkNamedColors()
        self.inverted_camera_matrices = inverted_camera_matrices
        self.output_path = output_path
        self.initial_frame_index = initial_frame_index
        self.frame_ray_datasets = frame_ray_datasets

        self.current_frame_index = initial_frame_index

        self.live_source_point_cloud = AllocationRaySourcePointCloud("Orange")
        self.canonical_source_point_cloud = AllocationRaySourcePointCloud("Peacock")

        self.segment_data = vtk.vtkPolyData()
        self.segment_mapper = vtk.vtkPolyDataMapper()
        self.segment_mapper.SetInputData(self.segment_data)
        self.segment_actor = vtk.vtkActor()
        self.segment_actor.SetMapper(self.segment_mapper)

        # TODO
        # renderer.AddActor(self.segment_actor)
        renderer.AddActor(self.live_source_point_cloud.get_actor())
        renderer.AddActor(self.canonical_source_point_cloud.get_actor())
        self.renderer = renderer

        # customizable initial visibility toggles
        self.live_source_point_cloud.toggle_visibility()

    def set_frame(self, frame_index):
        print(frame_index, len(self.frame_ray_datasets), self.initial_frame_index)
        if self.initial_frame_index < frame_index != self.current_frame_index:
            dataset = self.frame_ray_datasets[frame_index - self.initial_frame_index]

            self.live_source_point_cloud.update_points(dataset.live_surface_points)
            self.canonical_source_point_cloud.update_points(dataset.canonical_surface_points)

            segment_points = vtk.vtkPoints()
            lines = vtk.vtkCellArray()

            point_id = 0
            for x0, y0, z0, x1, y1, z1 in dataset.march_segment_endpoints:
                # print(x0, y0, z0, "|", x1, y1, z1)
                segment_points.InsertNextPoint(x0, -y0, z0)
                # points.InsertNextPoint(x0 + 0.01, -y0 + 0.01, z0 + 0.01)
                segment_points.InsertNextPoint(x1, -y1, z1)
                # segment_points.InsertNextPoint(x1 + 0.01, -y1 + 0.01, z1 + 0.01)
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
