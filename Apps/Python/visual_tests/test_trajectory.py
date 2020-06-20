#!/usr/bin/python3
import sys
import vtk
import numpy as np

from Apps.Python.visualizer import Mesh

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


class TestApp:
    def __init__(self):
        colors = vtk.vtkNamedColors()
        self._colors = colors

        # a renderer and render window
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(colors.GetColor3d("Black"))
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.SetWindowName("Test")
        self.render_window.AddRenderer(self.renderer)
        self.render_window.SetPosition(5121, 75)
        # fullscreen
        self.render_window.SetSize(self.render_window.GetScreenSize())

        # an interactor
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.render_window)
        self.interactor.Initialize()

        self.interactor.AddObserver("KeyPressEvent", self.keypress)
        self.interactor.AddObserver("LeftButtonPressEvent", self.button_event)
        self.interactor.AddObserver("LeftButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("RightButtonPressEvent", self.button_event)
        self.interactor.AddObserver("RightButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MiddleButtonPressEvent", self.button_event)
        self.interactor.AddObserver("MiddleButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MiddleButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MouseWheelForwardEvent", self.button_event)
        self.interactor.AddObserver("MouseWheelBackwardEvent", self.button_event)
        self.interactor.AddObserver("MouseMoveEvent", self.mouse_move)

        self.rotating = False
        self.panning = False
        self.zooming = False

        # add the actors to the scene
        self.draw_trajectory()

        self.mesh = Mesh(self.renderer, self.render_window, colors.GetColor3d("Peacock"))
        self.mesh.update("/mnt/Data/Reconstruction/experiment_output/2020-05-19/recording/Frame_17/canonical.ply")

        # axes actor
        self.axes = vtk.vtkAxesActor()
        self.axes_widget = widget = vtk.vtkOrientationMarkerWidget()
        rgba = colors.GetColor4ub("Carrot")
        widget.SetOutlineColor(rgba[0], rgba[1], rgba[2])
        widget.SetOrientationMarker(self.axes)
        widget.SetInteractor(self.interactor)
        widget.SetViewport(0.0, 0.2, 0.2, 0.4)
        widget.SetEnabled(1)
        widget.InteractiveOn()

        self.camera = camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0.2090549895511249, 1.213523311640004, -1.1883496392331843)
        camera.SetViewUp(0, 0, 0)
        camera.SetFocalPoint(0, 0, 0.512)
        camera.SetClippingRange(0.01, 10.0)
        # self.renderer.SetLightFollowCamera(False)
        # self.light = vtk.vtkLight()
        # self.light.SetLightTypeToSceneLight()
        # self.light.SetPosition(0.2090549895511249 - 0.5, 1.213523311640004, -1.1883496392331843)
        # self.light.SetPositional(True)
        # self.light.SetFocalPoint(0, 0, 0.512)
        # self.light_actor = vtk.vtkLightActor()
        # self.light_actor.SetLight(self.light)
        # self.renderer.AddViewProp(self.light_actor)


        self.render_window.Render()

    def draw_trajectory(self):
        viewpoints = np.array(
            [[0, 0, 0],
             [-0.588401, 0.0153606, 0.167214],
             [-0.885727, 0.0642821, 0.69977],
             [-0.717809, 0.118107, 1.2857],
             [-0.183011, 0.145305, 1.58178],
             [0.40539, 0.129945, 1.41457],
             [0.702716, 0.0810233, 0.882013],
             [0.534798, 0.0271984, 0.29608]], dtype=np.float32
        )
        self.draw_axes_array(viewpoints, self._colors.GetColor3d("Green"))
        directions = np.array(
            [[-0.114457, 0.0908755, 0.989263],
             [0.0331258, 0.0870227, 0.947322],
             [0.107701, 0.0747522, 0.813747],
             [0.065584, 0.0612519, 0.666783],
             [-0.0685541, 0.05443, 0.59252],
             [-0.216137, 0.0582827, 0.634461],
             [-0.290712, 0.0705532, 0.768036],
             [-0.248595, 0.0840536, 0.915]], dtype=np.float32
        )
        self.draw_point_array(directions, self._colors.GetColor3d("Orange"))

    def draw_point_array(self, point_array, color):
        points_vtk = vtk.vtkPoints()
        for x, y, z in point_array:
            points_vtk.InsertNextPoint(x, -y, z)

        points_polydata = vtk.vtkPolyData()
        points_polydata.SetPoints(points_vtk)

        points_filter = vtk.vtkVertexGlyphFilter()
        points_filter.SetInputData(points_polydata)

        points_mapper = vtk.vtkPolyDataMapper()
        points_mapper.SetInputConnection(points_filter.GetOutputPort())

        points_actor = vtk.vtkActor()
        points_actor.SetMapper(points_mapper)
        points_actor.GetProperty().SetPointSize(6)
        points_actor.GetProperty().SetColor(color)

        self.renderer.AddActor(points_actor)

    def draw_axes_array(self, point_array, color):
        points_vtk = vtk.vtkPoints()
        for x, y, z in point_array:
            points_vtk.InsertNextPoint(x, -y, z)

        points_polydata = vtk.vtkPolyData()
        points_polydata.SetPoints(points_vtk)

        axes = vtk.vtkAxes()
        axes.SetScaleFactor(0.5)
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(0.01)
        normals_filter = vtk.vtkPolyDataNormals()
        normals_filter.FlipNormalsOn()
        normals_filter.SetInputConnection(axes.GetOutputPort())

        axes_glyph_mapper = vtk.vtkGlyph3DMapper()
        axes_glyph_mapper.SetInputData(points_polydata)
        axes_glyph_mapper.SetSourceConnection(axes.GetOutputPort())

        axes_glyph_actor = vtk.vtkActor()
        axes_glyph_actor.SetMapper(axes_glyph_mapper)
        axes_glyph_actor.GetProperty().SetPointSize(6)
        axes_glyph_actor.GetProperty().SetColor(color)
        axes_glyph_actor.GetProperty().SetRepresentationToWireframe()

        self.renderer.AddActor(axes_glyph_actor)

    def launch(self):
        # begin mouse interaction
        self.interactor.Start()

    def button_event(self, obj, event):
        if event == "LeftButtonPressEvent":
            self.rotating = True
        elif event == "LeftButtonReleaseEvent":
            self.rotating = False
        elif event == "RightButtonPressEvent":
            self.panning = True
        elif event == "RightButtonReleaseEvent":
            self.panning = False
        elif event == "MiddleButtonPressEvent":
            self.zooming = True
        elif event == "MiddleButtonReleaseEvent":
            self.zooming = False
        elif event == "MouseWheelForwardEvent":
            self.dolly_step(1)
        elif event == "MouseWheelBackwardEvent":
            self.dolly_step(-1)

    # General high-level logic
    def mouse_move(self, obj, event):
        last_x_y_pos = self.interactor.GetLastEventPosition()
        last_x = last_x_y_pos[0]
        last_y = last_x_y_pos[1]

        x_y_pos = self.interactor.GetEventPosition()
        x = x_y_pos[0]
        y = x_y_pos[1]

        center = self.render_window.GetSize()
        center_x = center[0] / 2.0
        center_y = center[1] / 2.0

        if self.rotating:
            self.rotate(x, y, last_x, last_y)
        elif self.panning:
            self.pan(x, y, last_x, last_y, center_x, center_y)
        elif self.zooming:
            self.dolly(x, y, last_x, last_y)

    def rotate(self, x, y, last_x, last_y):
        speed = 0.5
        self.camera.Azimuth(speed * (last_x - x))
        self.camera.Elevation(speed * (last_y - y))
        self.camera.SetViewUp(0, 0, 0)
        self.render_window.Render()

    def pan(self, x, y, last_x, last_y, center_x, center_y):
        renderer = self.renderer
        camera = self.camera
        f_point = camera.GetFocalPoint()
        f_point0 = f_point[0]
        f_point1 = f_point[1]
        f_point2 = f_point[2]

        p_point = camera.GetPosition()
        p_point0 = p_point[0]
        p_point1 = p_point[1]
        p_point2 = p_point[2]

        renderer.SetWorldPoint(f_point0, f_point1, f_point2, 1.0)
        renderer.WorldToDisplay()
        d_point = renderer.GetDisplayPoint()
        focal_depth = d_point[2]

        a_point0 = center_x + (x - last_x)
        a_point1 = center_y + (y - last_y)

        renderer.SetDisplayPoint(a_point0, a_point1, focal_depth)
        renderer.DisplayToWorld()
        r_point = renderer.GetWorldPoint()
        r_point0 = r_point[0]
        r_point1 = r_point[1]
        r_point2 = r_point[2]
        r_point3 = r_point[3]

        if r_point3 != 0.0:
            r_point0 = r_point0 / r_point3
            r_point1 = r_point1 / r_point3
            r_point2 = r_point2 / r_point3

        camera.SetFocalPoint((f_point0 - r_point0) / 2.0 + f_point0,
                             (f_point1 - r_point1) / 2.0 + f_point1,
                             (f_point2 - r_point2) / 2.0 + f_point2)
        camera.SetPosition((f_point0 - r_point0) / 2.0 + p_point0,
                           (f_point1 - r_point1) / 2.0 + p_point1,
                           (f_point2 - r_point2) / 2.0 + p_point2)
        self.render_window.Render()

    def dolly(self, x, y, last_x, last_y):
        dolly_factor = pow(1.02, (0.5 * (y - last_y)))
        camera = self.camera
        if camera.GetParallelProjection():
            parallel_scale = camera.GetParallelScale() * dolly_factor
            camera.SetParallelScale(parallel_scale)
        else:
            camera.Dolly(dolly_factor)
            self.renderer.ResetCameraClippingRange()

        self.render_window.Render()

    def dolly_step(self, step):
        dolly_factor = pow(1.02, (10.0 * step))
        camera = self.camera
        if camera.GetParallelProjection():
            parallel_scale = camera.GetParallelScale() * dolly_factor
            camera.SetParallelScale(parallel_scale)
        else:
            camera.Dolly(dolly_factor)
            self.renderer.ResetCameraClippingRange()

        self.render_window.Render()

    def keypress(self, obj, event):
        key = obj.GetKeySym()
        print("Key:", key)
        if key == "q" or key == "Escape":
            obj.InvokeEvent("DeleteAllObjects")
            sys.exit()
        elif key == "c":
            print("Active camera position & orientation:")
            print(self.renderer.GetActiveCamera().GetPosition())
            print(self.renderer.GetActiveCamera().GetOrientation())
        elif key == "p":
            print(self.render_window.GetPosition())
        elif key == "m":
            self.mesh.toggle_visibility()
            self.render_window.Render()


def main():
    app = TestApp()
    app.launch()
    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
