#!/usr/bin/python3
import os
import sys
import gzip
import numpy as np
import vtk
from plyfile import PlyData, PlyElement

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


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


class Visualizer:
    BLOCK_SIZE = 8.0 * 0.004
    BLOCK_OFFSET = 0.002

    def __init__(self, allocation_sets):
        self.current_frame = 0
        self.frame_count = len(allocation_sets)

        self.allocation_sets = allocation_sets
        self.offset_cam = (0.2562770766576, 0.13962609403401335, -0.2113334598208764)

        colors = vtk.vtkNamedColors()

        self.allocation_block_points = vtk.vtkPoints()

        self.allocation_polydata = vtk.vtkPolyData()
        self.allocation_polydata.SetPoints(self.allocation_block_points)

        self.block_mapper = vtk.vtkGlyph3DMapper()
        self.block_mapper.SetInputData(self.allocation_polydata)

        block = vtk.vtkCubeSource()
        block.SetXLength(Visualizer.BLOCK_SIZE)
        block.SetYLength(Visualizer.BLOCK_SIZE)
        block.SetZLength(Visualizer.BLOCK_SIZE)
        self.block_mapper.SetSourceConnection(block.GetOutputPort())

        # The actor is a grouping mechanism: besides the geometry (mapper), it
        # also has a property, transformation matrix, and/or texture map.
        # Here we set its color and rotate it -22.5 degrees.
        self.block_actor = vtk.vtkActor()
        self.block_actor.SetMapper(self.block_mapper)
        self.block_actor.GetProperty().SetColor(colors.GetColor3d("PowderBlue"))
        self.block_actor.GetProperty().SetLineWidth(0.1)
        self.block_actor.GetProperty().SetRepresentationToWireframe()
        # self.block_actor.SetVisibility(False)

        # Create the graphics structure. The renderer renders into the render
        # window. The render window interactor captures mouse events and will
        # perform appropriate camera or actor manipulation depending on the
        # nature of the events.
        self.renderer = vtk.vtkRenderer()
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        # key_press_interactor_style = vtk.vtkKeyPressInteractorStyle()
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.render_window)

        # Add the actors to the renderer, set the background and size
        self.renderer.AddActor(self.block_actor)
        self.renderer.SetBackground(colors.GetColor3d("Black"))
        self.render_window.SetSize(1400, 900)
        self.render_window.SetWindowName('Allocation')

        # This allows the interactor to initalize itself. It has to be
        # called before an event loop.
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

        # We'll zoom in a little by accessing the camera and invoking a "Zoom"
        # method on it.
        self.camera = camera = self.renderer.GetActiveCamera()

        camera.SetPosition(self.offset_cam[0], self.offset_cam[1], self.offset_cam[2])
        camera.SetViewUp(0, 0, 0)
        camera.SetFocalPoint(0, 0, 0.512)
        camera.SetClippingRange(0.01, 10.0)

        self.canonical_mesh = Mesh(self.renderer, self.render_window, colors.GetColor3d("Cyan"))
        self.raw_live_mesh = Mesh(self.renderer, self.render_window, colors.GetColor3d("Pink"))
        self.warped_live_mesh = Mesh(self.renderer, self.render_window, colors.GetColor3d("Green"))
        self.shown_mesh_index = 0

        self.meshes = [self.raw_live_mesh, self.warped_live_mesh, self.canonical_mesh]
        self.mesh_names = ["canonical_mesh", "raw_live_mesh", "warped_live_mesh"]

        self.render_window.Render()
        self.set_frame(0)
        self.show_mesh_at_index(0)

    def load_frame_meshes(self, i_frame):
        base_path = "/mnt/Data/Reconstruction/experiment_output/2020-04-28/full_run"
        frame_path = os.path.join(base_path, "Frame_{:02}".format(i_frame + 16))
        canonical_path = os.path.join(frame_path, "canonical.ply")
        raw_live_path = os.path.join(frame_path, "live_raw.ply")
        warped_live_path = os.path.join(frame_path, "live_warped.ply")
        self.canonical_mesh.update(canonical_path)
        self.raw_live_mesh.update(raw_live_path)
        self.warped_live_mesh.update(warped_live_path)

    def launch(self):
        # Start the event loop.
        self.interactor.Start()

    def show_mesh_at_index(self, i_mesh_to_show):
        print("Mesh:", self.mesh_names[i_mesh_to_show])
        self.shown_mesh_index = i_mesh_to_show
        i_mesh = 0
        for mesh in self.meshes:
            if i_mesh_to_show == i_mesh:
                mesh.show()
            else:
                mesh.hide()
            i_mesh += 1
        self.render_window.Render()

    def set_frame(self, i_frame):
        print("Frame:", i_frame + 16)

        self.load_frame_meshes(i_frame)

        self.current_frame = i_frame
        allocation_data = self.allocation_sets[i_frame]
        del self.allocation_block_points
        self.allocation_block_points = vtk.vtkPoints()

        for block_coordinate in allocation_data:
            point_scaled = block_coordinate * Visualizer.BLOCK_SIZE - Visualizer.BLOCK_OFFSET
            # print(point_scaled)
            self.allocation_block_points.InsertNextPoint(point_scaled[0], -point_scaled[1], point_scaled[2])
            # (point_scaled[0] * Visualizer.BLOCK_SIZE, point_scaled[1] * Visualizer.BLOCK_SIZE, point_scaled[2] * Visualizer.BLOCK_SIZE))

        self.allocation_polydata.SetPoints(self.allocation_block_points)
        self.block_mapper.SetInputData(self.allocation_polydata)
        self.block_mapper.Modified()
        self.render_window.Render()

    def advance_view(self):
        if self.shown_mesh_index == len(self.meshes) - 1:
            if self.current_frame < self.frame_count - 1:
                self.set_frame(self.current_frame + 1)
                self.show_mesh_at_index(0)
        else:
            self.show_mesh_at_index(self.shown_mesh_index + 1)

    def retreat_view(self):
        if self.shown_mesh_index == 0:
            if self.current_frame > 0:
                self.set_frame(self.current_frame - 1)
                self.show_mesh_at_index(len(self.meshes) - 1)
        else:
            self.show_mesh_at_index(self.shown_mesh_index - 1)

    # Handle the mouse button events.
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
            self.dolly(x, y, last_x, last_y, center_x, center_y)

    def rotate(self, x, y, last_x, last_y):
        speed = 0.5
        self.camera.Azimuth(speed * (last_x - x))
        self.camera.Elevation(speed * (last_y - y))
        self.camera.SetViewUp(0, 0, 0)
        self.render_window.Render()

    def pan(self, x, y, lastX, lastY, centerX, centerY):
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

        a_point0 = centerX + (x - lastX)
        a_point1 = centerY + (y - lastY)

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

    def dolly(self, x, y, lastX, lastY, centerX, centerY):
        dolly_factor = pow(1.02, (0.5 * (y - lastY)))
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
        elif key == "bracketright":
            if self.current_frame < self.frame_count - 1:
                self.set_frame(self.current_frame + 1)
        elif key == "bracketleft":
            if self.current_frame > 0:
                self.set_frame(self.current_frame - 1)
        elif key == "c":
            print(self.renderer.GetActiveCamera().GetPosition())
        elif key == "Right":
            self.advance_view()
        elif key == "Left":
            self.retreat_view()


def read_allocation_data(
        data_path="/mnt/Data/Reconstruction/experiment_output/2020-04-28/full_run/canonical_volume_memory_usage.dat"):
    file = gzip.open(data_path, "rb")

    point_sets = []
    i_frame = 17
    while file.readable():
        buffer = file.read(size=8)
        if not buffer:
            break
        count = np.frombuffer(buffer, dtype=np.uint32)[0]
        print("reading frame", i_frame, "of size", count, "...")
        point_set = np.resize(np.frombuffer(file.read(size=6 * count), dtype=np.int16), (count, 3))
        point_sets.append(point_set)
        i_frame += 1
    return point_sets


def main():
    allocation_sets = read_allocation_data()
    visualizer = Visualizer(allocation_sets)

    visualizer.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
