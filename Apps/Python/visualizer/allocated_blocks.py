import os

import vtk
import gzip
import numpy as np

from Apps.Python.visualizer.geometric_conversions import convert_block_to_metric, VoxelVolumeParameters


def read_allocation_data(data_path, i_frame):
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
        block_sets.append((block_set, metric_set))
        i_frame += 1
    return block_sets


class AllocatedBlocks:

    def __init__(self, renderer, output_path, start_frame_index):
        self.start_frame_index = start_frame_index
        self.output_path = output_path
        self.renderer = renderer
        allocated_block_sets = read_allocation_data(os.path.join(output_path, "canonical_volume_memory_usage.dat"),
                                                    start_frame_index)
        self.frame_count = len(allocated_block_sets)
        self.allocated_block_sets = allocated_block_sets

        colors = vtk.vtkNamedColors()

        self.block_locations = vtk.vtkPoints()
        self.block_labels = vtk.vtkStringArray()
        self.block_labels.SetName("label")
        self.verts = vtk.vtkCellArray()

        self.block_polydata = vtk.vtkPolyData()
        self.block_polydata.SetPoints(self.block_locations)
        self.block_polydata.SetVerts(self.verts)
        self.block_polydata.GetPointData().AddArray(self.block_labels)

        self.block_label_hierarchy = vtk.vtkPointSetToLabelHierarchy()
        self.block_label_hierarchy.SetInputData(self.block_polydata)
        self.block_label_hierarchy.SetLabelArrayName("label")

        self.block_label_placement_mapper = vtk.vtkLabelPlacementMapper()
        self.block_label_placement_mapper.SetInputConnection(self.block_label_hierarchy.GetOutputPort())
        self.block_label_placement_mapper.SetRenderStrategy(vtk.vtkFreeTypeLabelRenderStrategy())
        self.block_label_placement_mapper.SetShapeToRoundedRect()
        self.block_label_placement_mapper.SetBackgroundColor(1.0, 1.0, 0.7)
        self.block_label_placement_mapper.SetBackgroundOpacity(0.4)
        self.block_label_placement_mapper.SetMargin(3)

        self.block_mapper = vtk.vtkGlyph3DMapper()
        self.block_mapper.SetInputData(self.block_polydata)

        block = vtk.vtkCubeSource()
        block.SetXLength(VoxelVolumeParameters.BLOCK_SIZE)
        block.SetYLength(VoxelVolumeParameters.BLOCK_SIZE)
        block.SetZLength(VoxelVolumeParameters.BLOCK_SIZE)
        self.block_mapper.SetSourceConnection(block.GetOutputPort())

        # block actor
        self.block_actor = vtk.vtkActor()
        self.block_actor.SetMapper(self.block_mapper)
        self.block_actor.GetProperty().SetColor(colors.GetColor3d("PowderBlue"))
        self.block_actor.GetProperty().SetLineWidth(0.1)
        self.block_actor.GetProperty().SetRepresentationToWireframe()

        # label actor
        self.block_label_actor = vtk.vtkActor2D()
        self.block_label_actor.SetMapper(self.block_label_placement_mapper)

        self.renderer.AddActor(self.block_actor)
        self.renderer.AddActor(self.block_label_actor)

    def set_frame(self, i_frame):
        allocated_block_set, metric_set = self.allocated_block_sets[i_frame - self.start_frame_index]
        del self.block_locations
        self.block_locations = vtk.vtkPoints()
        self.block_labels.SetNumberOfValues(len(allocated_block_set))
        del self.verts
        self.verts = vtk.vtkCellArray()

        i_block = 0
        for block_coordinate in allocated_block_set:
            metric_coord = metric_set[i_block]
            self.block_locations.InsertNextPoint((metric_coord[0], -metric_coord[1], metric_coord[2]))

            label = "({:d}, {:d}, {:d})".format(block_coordinate[0],
                                                block_coordinate[1],
                                                block_coordinate[2])

            self.block_labels.SetValue(i_block, label)
            self.verts.InsertNextCell(1)
            self.verts.InsertCellPoint(i_block)
            i_block += 1

        self.block_polydata.SetPoints(self.block_locations)
        self.block_polydata.SetVerts(self.verts)

        self.block_mapper.SetInputData(self.block_polydata)
        self.block_label_placement_mapper.Modified()
        self.block_mapper.Modified()

    def toggle_labels(self):
        self.block_label_actor.SetVisibility(not self.block_label_actor.GetVisibility())

    def toggle_visibility(self):
        self.block_actor.SetVisibility(not self.block_actor.GetVisibility())
        pass