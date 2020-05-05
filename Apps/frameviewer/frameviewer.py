#!/usr/bin/python3
import sys
from enum import Enum

import cv2

import vtk
import numpy as np

from Apps.frameviewer import frameloading, image_conversion, trajectoryloading

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


class ViewingMode(Enum):
    DEPTH = 0
    COLOR = 1


class CameraProjection:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def project_to_camera_space(self, u, v, depth):
        x = (u * depth - self.cx) / self.fx
        y = (v * depth - self.cy) / self.fy
        return x, y, depth


class FrameViewerApp:
    PROJECTION = CameraProjection(fx=517, fy=517, cx=320, cy=240)

    def __init__(self, output_folder, frame_index_to_start_with):
        self.start_frame_index = frame_index_to_start_with
        self.inverse_camera_matrices = trajectoryloading.load_inverse_matrices(output_folder)
        self.current_camera_matrix = None if len(self.inverse_camera_matrices) == 0 else self.inverse_camera_matrices[0]

        self.image_masks_enabled = True

        self.color_numpy_image = None
        self.depth_numpy_image = None
        self.depth_numpy_image_uint8 = None

        self.scaled_color = None
        self.scaled_depth = None
        self.color_vtk_image = None
        self.depth_vtk_image = None
        self.image_size = None

        self.image_mapper = vtk.vtkImageMapper()
        self.image_mapper.SetColorWindow(255)
        self.image_mapper.SetColorLevel(127.5)

        self.image_actor = vtk.vtkActor2D()
        self.image_actor.SetMapper(self.image_mapper)
        self.image_actor.SetPosition(20, 20)

        colors = vtk.vtkNamedColors()
        window_width = 1400
        window_height = 900

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.SetSize(window_width, window_height)
        self.render_window.AddRenderer(self.renderer)

        self.renderer.AddActor2D(self.image_actor)

        self.frame_index = -1
        self.viewing_mode = ViewingMode.COLOR
        self.scale = 2.0
        self._scaled_resolution = None
        self.panning = False
        self.zooming = False

        self.text_mapper = vtk.vtkTextMapper()
        self.text_mapper.SetInput("Frame: {:d} | Scale: {:f}\nPixel: 0, 0\nDepth: 0 m\nColor: 0 0 0\n"
                                  "Camera-space: 0 0 0\nWorld-space: 0 0 0"
                                  .format(frame_index_to_start_with, self.scale))
        self.text_mapper.GetInput()
        number_of_lines = len(self.text_mapper.GetInput().splitlines())
        text_property = self.text_mapper.GetTextProperty()
        font_size = 15
        text_property.SetFontSize(font_size)
        text_property.SetColor(colors.GetColor3d('Cyan'))

        self.text_actor = vtk.vtkActor2D()
        self.text_actor.SetMapper(self.text_mapper)
        self.text_actor.SetDisplayPosition(30, window_height - 10 - number_of_lines * font_size)
        self.renderer.AddActor(self.text_actor)

        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.render_window)
        self.interactor.AddObserver("KeyPressEvent", self.keypress)
        self.interactor.AddObserver("LeftButtonPressEvent", self.button_event)
        self.interactor.AddObserver("LeftButtonReleaseEvent", self.button_event)
        self.interactor.AddObserver("MouseWheelForwardEvent", self.button_event)
        self.interactor.AddObserver("MouseWheelBackwardEvent", self.button_event)
        self.interactor.AddObserver("MouseMoveEvent", self.mouse_move)

        self.frame_index = None
        self.set_frame(frame_index_to_start_with)

    def launch(self):
        self.interactor.Initialize()
        self.render_window.Render()
        self.interactor.Start()

    def update_active_vtk_image(self, force_reset=False):
        data_by_mode = {
            ViewingMode.COLOR: (self.scaled_color, self.color_vtk_image),
            ViewingMode.DEPTH: (self.scaled_depth, self.depth_vtk_image)
        }
        numpy_image_source, vtk_image_target = data_by_mode[self.viewing_mode]

        if vtk_image_target is None or force_reset is True:
            vtk_image_target = image_conversion.numpy_image_as_vtk_image_data(numpy_image_source)
        else:
            image_conversion.update_vtk_image(vtk_image_target, numpy_image_source)

        self.image_mapper.SetInputData(vtk_image_target)
        if self.viewing_mode == ViewingMode.DEPTH:
            self.depth_vtk_image = vtk_image_target
        elif self.viewing_mode == ViewingMode.COLOR:
            self.color_vtk_image = vtk_image_target

        self.render_window.Render()

    def update_scaled_images(self):
        self._scaled_resolution = (self.image_size * self.scale).astype(np.int32)
        if self.scale % 1.0 == 0:
            interpolation_mode = cv2.INTER_NEAREST
        else:
            interpolation_mode = cv2.INTER_LINEAR
        self.scaled_depth = cv2.resize(self.depth_numpy_image_uint8, tuple(self._scaled_resolution),
                                       interpolation=interpolation_mode)
        self.scaled_color = cv2.resize(self.color_numpy_image, tuple(self._scaled_resolution),
                                       interpolation=interpolation_mode)
        self.update_active_vtk_image(force_reset=True)

    def update_viewing_mode(self, viewing_mode):
        """
        :type viewing_mode ViewingMode
        """
        if self.viewing_mode != viewing_mode:
            print("Viewing mode:", viewing_mode.name)
            self.viewing_mode = viewing_mode
            self.update_active_vtk_image()

    def set_frame(self, frame_index):
        print("Frame:", frame_index)
        self.current_camera_matrix = None if len(self.inverse_camera_matrices) <= frame_index \
            else self.inverse_camera_matrices[frame_index-self.start_frame_index]

        self.frame_index = frame_index
        self.color_numpy_image = frameloading.load_color_numpy_image(frame_index)
        self.depth_numpy_image = frameloading.load_depth_numpy_image(frame_index)
        self.image_size = np.array((self.color_numpy_image.shape[1], self.color_numpy_image.shape[0]))

        if self.image_masks_enabled:
            mask_image = frameloading.load_mask_numpy_image(frame_index)
            self.color_numpy_image[mask_image == 0] = 0
            self.depth_numpy_image[mask_image == 0] = 0

        self.depth_numpy_image_uint8 = image_conversion.convert_to_viewable_depth(self.depth_numpy_image)
        self.update_scaled_images()

    def zoom_scale(self, step, increment_step=False, increment=0.5):
        x_y_pos = self.interactor.GetEventPosition()
        x = x_y_pos[0]
        y = x_y_pos[1]
        if increment_step:
            self.scale -= (self.scale % increment)
            self.scale += (step * increment)
        else:
            self.scale *= pow(1.02, (1.0 * step))
        self.update_scaled_images()
        self.report_on_mouse_location(x, y)

    # Handle the mouse button events.
    def button_event(self, obj, event):
        if event == "LeftButtonPressEvent":
            self.panning = True
        elif event == "LeftButtonReleaseEvent":
            self.panning = False
        elif event == "RightButtonPressEvent":
            self.zooming = True
        elif event == "RightButtonReleaseEvent":
            self.zooming = False
        elif event == "MouseWheelForwardEvent":
            self.zoom_scale(1, self.interactor.GetControlKey())
        elif event == "MouseWheelBackwardEvent":
            self.zoom_scale(-1, self.interactor.GetControlKey())

    def keypress(self, obj, event):
        key = obj.GetKeySym()
        print("Key:", key)
        if key == "q" or key == "Escape":
            obj.InvokeEvent("DeleteAllObjects")
            sys.exit()
        elif key == "bracketright":
            self.set_frame(self.frame_index + 1)
        elif key == "bracketleft":
            self.set_frame(self.frame_index - 1)
        elif key == "c":
            pass
        elif key == "Right":
            pass
        elif key == "Left":
            pass

    def mouse_move(self, obj, event):
        last_x_y_pos = self.interactor.GetLastEventPosition()
        last_x = last_x_y_pos[0]
        last_y = last_x_y_pos[1]

        x_y_pos = self.interactor.GetEventPosition()
        x = x_y_pos[0]
        y = x_y_pos[1]

        if self.panning:
            self.pan(x, y, last_x, last_y)
        elif self.zooming:
            self.zoom(x, y, last_x, last_y)
        else:
            self.report_on_mouse_location(x, y)

    def zoom(self, x, y, last_x, last_y):
        self.scale *= pow(1.02, (0.5 * (y - last_y)))
        self.update_scaled_images()

    def pan(self, x, y, last_x, last_y):
        point_x = x - last_x
        point_y = y - last_y
        image_position = self.image_actor.GetPosition()
        self.image_actor.SetPosition(image_position[0] + point_x, image_position[1] + point_y)
        self.render_window.Render()

    def is_pixel_within_image(self, x, y):
        image_start_x, image_start_y = self.image_actor.GetPosition()
        image_end_x = image_start_x + self._scaled_resolution[0]
        image_end_y = image_start_y + self._scaled_resolution[1]
        return image_start_x < x < image_end_x and image_start_y < y < image_end_y

    def get_frame_pixel(self, x, y):
        image_start_x, image_start_y = self.image_actor.GetPosition()
        frame_x = int((x - image_start_x) / self.scale + 0.5)
        frame_y = self.color_numpy_image.shape[0] - int((y - image_start_y) / self.scale + 0.5)
        return frame_x, frame_y

    def update_location_text(self, u, v, depth, color):
        camera_coords = FrameViewerApp.PROJECTION.project_to_camera_space(u, v, depth)
        camera_coords_homogenized = np.array((camera_coords[0], camera_coords[1], camera_coords[2], 1.0)).T
        world_coords = self.current_camera_matrix.dot(camera_coords_homogenized)
        # world_coords /= world_coords[3]
        # print(self.current_camera_matrix)
        self.text_mapper.SetInput(
            "Frame: {:d} | Scale: {:f}\nPixel: {:d}, {:d}\nDepth: {:f} m\nColor: {:d}, {:d}, {:d}\n"
            "Camera-space: {:02.4f}, {:02.4f}, {:02.4f}\nWorld-space: {:02.4f}, {:02.4f}, {:02.4f}"
            .format(self.frame_index, self.scale, u, v, depth, color[0], color[1], color[2],
                    camera_coords[0], camera_coords[1], camera_coords[2],
                    world_coords[0], world_coords[1], world_coords[2]))
        self.text_mapper.Modified()
        self.render_window.Render()

    def report_on_mouse_location(self, x, y):
        if self.is_pixel_within_image(x, y):
            frame_x, frame_y = self.get_frame_pixel(x, y)
            depth = self.depth_numpy_image[frame_y, frame_x]
            color = self.color_numpy_image[frame_y, frame_x]
            self.update_location_text(frame_x, frame_y, depth, color)


def main():
    app = FrameViewerApp("/mnt/Data/Reconstruction/experiment_output/2020-05-04/recording", 16)
    app.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
