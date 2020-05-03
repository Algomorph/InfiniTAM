#!/usr/bin/python3
import sys
from enum import Enum

import cv2

import vtk
import numpy as np

from Apps.frameviewer import frameloading, image_conversion

PROGRAM_EXIT_SUCCESS = 0
PROGRAM_EXIT_FAILURE = -1


class ViewingMode(Enum):
    DEPTH = 0
    COLOR = 1


class FrameViewerApp:
    NEAR_CLIPPING_DISTANCE = 0.2
    FAR_CLIPPING_DISTANCE = 3.0
    CLIPPING_RANGE = FAR_CLIPPING_DISTANCE - NEAR_CLIPPING_DISTANCE

    def __init__(self, frame_index_to_start_with=16):
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

        image_actor = vtk.vtkActor2D()
        image_actor.SetMapper(self.image_mapper)
        image_actor.SetPosition(20, 20)

        colors = vtk.vtkNamedColors()
        window_width = 1400
        window_height = 900

        self.text_mapper = vtk.vtkTextMapper()
        self.text_mapper.SetInput("Frame: 16\n Pixel: 0, 0\nDepth: 0 m")
        number_of_lines = 3
        text_property = self.text_mapper.GetTextProperty()
        font_size = 15
        text_property.SetFontSize(font_size)
        text_property.SetColor(colors.GetColor3d('Cyan'))

        self.text_actor = vtk.vtkActor2D()
        self.text_actor.SetMapper(self.text_mapper)
        self.text_actor.SetDisplayPosition(30,window_height - 10 - number_of_lines * font_size)

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.SetSize(window_width, window_height)
        self.render_window.AddRenderer(self.renderer)

        self.renderer.AddActor2D(image_actor)

        self.frame_index = -1
        self.viewing_mode = ViewingMode.COLOR
        self.scale = 2.0
        self.panning = False
        self.zooming = False

        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetInteractorStyle(None)
        self.interactor.SetRenderWindow(self.render_window)
        self.interactor.AddObserver("KeyPressEvent", self.keypress)
        self.interactor.AddObserver("MouseWheelForwardEvent", self.button_event)
        self.interactor.AddObserver("MouseWheelBackwardEvent", self.button_event)

        self.renderer.AddActor(self.text_actor)

        self.frame_index = None
        self.load_frame_images(frame_index_to_start_with)

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
        self.scaled_depth = cv2.resize(self.depth_numpy_image_uint8,
                                       tuple((self.image_size * self.scale).astype(np.int32)))
        self.scaled_color = cv2.resize(self.color_numpy_image, tuple((self.image_size * self.scale).astype(np.int32)))
        self.update_active_vtk_image(force_reset=True)

    def update_viewing_mode(self, viewing_mode):
        """
        :type viewing_mode ViewingMode
        """
        if self.viewing_mode != viewing_mode:
            print("Viewing mode:", viewing_mode.name)
            self.viewing_mode = viewing_mode
            self.update_active_vtk_image()

    def load_frame_images(self, frame_index):
        print("Frame:", frame_index)
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

    def zoom_scale(self, factor):
        self.scale *= pow(1.02, (1.0 * factor))
        self.update_scaled_images()

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
            self.zoom_scale(1)
        elif event == "MouseWheelBackwardEvent":
            self.zoom_scale(-1)

    def keypress(self, obj, event):
        key = obj.GetKeySym()
        print("Key:", key)
        if key == "q" or key == "Escape":
            obj.InvokeEvent("DeleteAllObjects")
            sys.exit()
        elif key == "bracketright":
            self.load_frame_images(self.frame_index + 1)
        elif key == "bracketleft":
            self.load_frame_images(self.frame_index - 1)
        elif key == "c":
            pass
        elif key == "Right":
            pass
        elif key == "Left":
            pass


def main():
    app = FrameViewerApp()
    app.launch()

    return PROGRAM_EXIT_SUCCESS


if __name__ == "__main__":
    sys.exit(main())
