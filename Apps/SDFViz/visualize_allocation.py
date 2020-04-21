#!/usr/bin/python3

import sys
import gzip
import numpy as np
import vtk

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1

offset_cam = (-500, -200, -500)


def visualize_point_set(point_set):
    colors = vtk.vtkNamedColors()

    # Set the background color.
    # bkg = map(lambda x: x / 255.0, [26, 51, 102, 255])
    # colors.SetColor("BkgColor", *bkg)

    # This creates a polygonal block model
    block = vtk.vtkCubeSource()
    block.SetXLength(1)
    block.SetYLength(1)
    block.SetZLength(1)

    points = vtk.vtkPoints()

    for point in point_set:
        # print(point)
        # points.InsertNextPoint(0, 1, 2)
        points.InsertNextPoint((point[0], point[1], point[2]))

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(points)

    pointMapper = vtk.vtkGlyph3DMapper()
    pointMapper.SetInputData(polyData)
    pointMapper.SetSourceConnection(block.GetOutputPort())

    # The actor is a grouping mechanism: besides the geometry (mapper), it
    # also has a property, transformation matrix, and/or texture map.
    # Here we set its color and rotate it -22.5 degrees.
    blockActor = vtk.vtkActor()
    blockActor.SetMapper(pointMapper)
    blockActor.GetProperty().SetColor(colors.GetColor3d("Tomato"))
    # blockActor.RotateX(30.0)
    # blockActor.RotateY(45.0)

    # Create the graphics structure. The renderer renders into the render
    # window. The render window interactor captures mouse events and will
    # perform appropriate camera or actor manipulation depending on the
    # nature of the events.
    renderer = vtk.vtkRenderer()
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    #key_press_interactor_style = vtk.vtkKeyPressInteractorStyle()
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)

    # Add the actors to the renderer, set the background and size
    renderer.AddActor(blockActor)
    renderer.SetBackground(colors.GetColor3d("Black"))
    render_window.SetSize(1400, 900)
    render_window.SetWindowName('Allocation')

    # This allows the interactor to initalize itself. It has to be
    # called before an event loop.
    interactor.Initialize()

    # We'll zoom in a little by accessing the camera and invoking a "Zoom"
    # method on it.
    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.Zoom(1.0)
    # offset = (-256, -256, 0)
    offset = (0, 0, 0)
    camera.SetPosition(offset_cam[0], offset_cam[1], offset_cam[2])
    print(camera.GetPosition())
    camera.SetViewUp(0, 1, 0)
    camera.SetFocalPoint(0 - offset[0], 0 - offset[1], offset[2])
    camera.SetClippingRange(1.0, 2000)

    render_window.Render()

    # Start the event loop.
    interactor.Start()


def main():
    data_path = "/mnt/Data/Reconstruction/experiment_output/2020-04-21/full_run/canonical_volume_memory_usage.dat"
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
    # visualize_point_set(point_sets[0])
    for point_set in point_sets:
        visualize_point_set(point_set)

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
