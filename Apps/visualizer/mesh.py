
import numpy as np
import vtk
from plyfile import PlyData, PlyElement


class Mesh:
    FACTOR = 1.0

    def __init__(self, renderer, render_window, color):
        self.renderer = renderer
        self.render_window = render_window

        # Create a polydata object
        self.triangle_poly_data = vtk.vtkPolyData()

        # Create mapper and actor
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.triangle_poly_data)
        self.actor = vtk.vtkActor()
        self.actor.GetProperty().SetColor(color)
        self.actor.GetProperty().SetBackfaceCulling(True)
        self.actor.SetMapper(self.mapper)
        self.renderer.AddActor(self.actor)

    def update(self, path, render=False):
        ply_data = PlyData.read(path)
        vertex_data = ply_data["vertex"]
        index_data = ply_data['face'].data['vertex_indices']
        points = vtk.vtkPoints()

        for vert_d in vertex_data:
            vert = np.array(vert_d.tolist()) * Mesh.FACTOR
            # print(vert)
            points.InsertNextPoint((vert[0], -vert[1], vert[2]))

        triangle_count = points.GetNumberOfPoints() // 3
        triangles = vtk.vtkCellArray()

        for i_tri in range(0, triangle_count):
            triangle = vtk.vtkTriangle()
            indices = index_data[i_tri]
            triangle.GetPointIds().SetId(0, indices[0])
            triangle.GetPointIds().SetId(1, indices[1])
            triangle.GetPointIds().SetId(2, indices[2])
            triangles.InsertNextCell(triangle)

        # Add the geometry and topology to the polydata
        self.triangle_poly_data.SetPoints(points)
        self.triangle_poly_data.SetPolys(triangles)

        self.mapper.SetInputData(self.triangle_poly_data)
        self.mapper.Modified()

        if render:
            self.render_window.Render()

    def hide(self):
        self.actor.SetVisibility(False)

    def show(self):
        self.actor.SetVisibility(True)