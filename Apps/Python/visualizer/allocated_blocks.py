from typing import List, TypeVar, Generic

import vtk

from Apps.Python.visualizer.alloction_data_processing import FrameBlockData, FramePixelBlockData
from Apps.Python.visualizer.geometric_conversions import VoxelVolumeParameters

TFrameBlockData = TypeVar('TFrameBlockData', FrameBlockData, FramePixelBlockData)


class AllocatedBlocks(Generic[TFrameBlockData]):
    def __init__(self, renderer, output_path, start_frame_index, block_sets: List[TFrameBlockData]):
        self.start_frame_index = start_frame_index
        self.output_path = output_path
        self.renderer = renderer

        self.frame_count = len(block_sets)
        self.block_sets = block_sets

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
        block_set = self.block_sets[i_frame - self.start_frame_index]
        del self.block_locations
        self.block_locations = vtk.vtkPoints()
        self.block_labels.SetNumberOfValues(len(block_set.block_coordinates))
        del self.verts
        self.verts = vtk.vtkCellArray()

        i_block = 0
        for metric_coord in block_set.metric_block_coordinates:
            self.block_locations.InsertNextPoint((metric_coord[0], -metric_coord[1], metric_coord[2]))
            label = block_set.make_label_text(i_block)
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

    def hide_labels(self):
        self.block_label_actor.SetVisibility(False)

    def show_labels(self):
        self.block_label_actor.SetVisibility(True)

    def labels_visible(self):
        return self.block_label_actor.GetVisibility()

    def toggle_visibility(self):
        self.block_actor.SetVisibility(not self.block_actor.GetVisibility())

    def hide_blocks(self):
        self.block_actor.SetVisibility(False)

    def show_blocks(self):
        self.block_actor.SetVisibility(True)

    def blocks_visible(self):
        return self.block_actor.GetVisibility()
