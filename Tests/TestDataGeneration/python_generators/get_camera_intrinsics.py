#  ================================================================
#  Created by Gregory Kramida on 1/28/19.
#  Copyright (c) 2019 Gregory Kramida
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#  ================================================================

import bpy
import numpy as np
import sys
import argparse
import os

parser = argparse.ArgumentParser("A script that computes the camera intrinsic matrix from a blender file.")

parser.add_argument("-i", "--input", help="The input blender file.",
                    default="CoffeeCup_Depth.blend")


def get_sensor_size(sensor_fit, sensor_x, sensor_y):
    if sensor_fit == 'VERTICAL':
        return sensor_y
    return sensor_x


def get_sensor_fit(sensor_fit, size_x, size_y):
    if sensor_fit == 'AUTO':
        if size_x >= size_y:
            return 'HORIZONTAL'
        else:
            return 'VERTICAL'
    return sensor_fit


# Build intrinsic camera parameters from Blender camera data
#
# See notes on this in
# blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model
# as well as
# https://blender.stackexchange.com/a/120063/3581
def get_camera_projection_from_blender(camera_data):
    if camera_data.type != 'PERSP':
        raise ValueError('Non-perspective cameras not supported')
    scene = bpy.context.scene
    f_in_mm = camera_data.lens
    print(f_in_mm)
    scale = scene.render.resolution_percentage / 100
    resolution_x_in_px = scale * scene.render.resolution_x
    resolution_y_in_px = scale * scene.render.resolution_y
    sensor_size_in_mm = get_sensor_size(camera_data.sensor_fit, camera_data.sensor_width, camera_data.sensor_height)
    sensor_fit = get_sensor_fit(
        camera_data.sensor_fit,
        scene.render.pixel_aspect_x * resolution_x_in_px,
        scene.render.pixel_aspect_y * resolution_y_in_px
    )
    pixel_aspect_ratio = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x
    if sensor_fit == 'HORIZONTAL':
        view_factor_in_px = resolution_x_in_px
    else:
        view_factor_in_px = pixel_aspect_ratio * resolution_y_in_px

    pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_factor_in_px
    f_x = 1 / pixel_size_mm_per_px
    f_y = 1 / pixel_size_mm_per_px / pixel_aspect_ratio

    # Parameters of intrinsic projection matrix
    c_x = resolution_x_in_px / 2 - camera_data.shift_x * view_factor_in_px
    c_y = resolution_y_in_px / 2 + camera_data.shift_y * view_factor_in_px / pixel_aspect_ratio
    skew = 0  # only use rectangular pixels

    camera_projection = np.array([[f_x, skew, c_x], [0, f_y, c_y], [0, 0, 1]])

    return camera_projection


EXIT_CODE_FAILURE = 1
EXIT_CODE_SUCCESS = 0


def main():
    exit_code = EXIT_CODE_SUCCESS
    args = parser.parse_args()
    blender_filename = os.path.basename(args.input)
    if not blender_filename.endswith(".blend"):
        print("Warning: the blender filename does not end with .blend. " +
              "Only valid blender files are supported.")
    bpy.ops.wm.open_mainfile(filepath=args.input)
    scene = bpy.context.scene
    camera_data = scene.camera.data
    camera_projection = get_camera_projection_from_blender(camera_data)
    print(camera_projection)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
