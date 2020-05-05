import vtk


class PixelHighlighter:
    def __init__(self, renderer):
        pixel_highlighter_points = vtk.vtkPoints()

        pixel_highlighter_points.InsertPoint(0, 0, 0, 0)
        pixel_highlighter_points.InsertPoint(1, 2, 0, 0)
        pixel_highlighter_points.InsertPoint(2, 2, 2, 0)
        pixel_highlighter_points.InsertPoint(3, 0, 2, 0)

        pixel_highlighter_polygon = vtk.vtkCellArray()
        pixel_highlighter_polygon.InsertNextCell(5)
        pixel_highlighter_polygon.InsertCellPoint(0)
        pixel_highlighter_polygon.InsertCellPoint(1)
        pixel_highlighter_polygon.InsertCellPoint(2)
        pixel_highlighter_polygon.InsertCellPoint(3)
        pixel_highlighter_polygon.InsertCellPoint(0)

        polygon_poly_data = vtk.vtkPolyData()
        polygon_poly_data.SetPoints(pixel_highlighter_points)
        polygon_poly_data.SetLines(pixel_highlighter_polygon)

        self.transform = vtk.vtkTransform()
        self.scale = 1.0
        self.transform.Scale(0.5, 0.5, 0.5)

        self.transform_filter = vtk.vtkTransformFilter()
        self.transform_filter.SetInputData(polygon_poly_data)
        self.transform_filter.SetTransform(self.transform)

        self.mapper = vtk.vtkPolyDataMapper2D()
        self.mapper.SetInputConnection(self.transform_filter.GetOutputPort())

        self.actor = vtk.vtkActor2D()
        self.actor.SetMapper(self.mapper)
        colors = vtk.vtkNamedColors()
        self.actor.GetProperty().SetColor(colors.GetColor3d("Green"))
        self.actor.SetPosition(0, 0)

        renderer.AddActor(self.actor)

    def nudge_scale(self, adjustment=0.5):
        new_scale = self.scale + adjustment
        if new_scale > 0.1:
            self.scale = new_scale
            self.set_scale(self.scale)

    def get_scale(self):
        return self.transform.GetScale()

    def set_scale(self, scale):
        current_scale = self.transform.GetScale()[0]
        adjustment = scale / current_scale * 0.5
        self.transform.Scale(adjustment, adjustment, adjustment)
        self.transform_filter.SetTransform(self.transform)
        self.mapper.Update()

    def set_position(self, x, y):
        self.actor.SetPosition(x, y)

    def hide(self):
        self.actor.SetVisibility(False)

    def show(self):
        self.actor.SetVisibility(True)
