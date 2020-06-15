#!/usr/bin/python3

# system
import glob
import re
import sys
import argparse
import os.path
import os
import struct
from math import pi
import shutil

# math libs
import numpy as np

# cv libs
import cv2

# blender
import bpy
import bpy.ops as bops
import bpy.types

# local
if "PYCHARM_HOSTED" in os.environ and os.environ["PYCHARM_HOSTED"]:
    from Tests.TestData.generators.utils import draw_contours, make_labeling_consistent
else:
    from utils import draw_contours, make_labeling_consistent


class RGBDCameraEmulatorApplication:
    # depth of mist, in meters: this will translate into total range of depth
    MIST_DEPTH_M = 10.0

    def __init__(self, args):
        self.scene_name = ""
        self.args = args

    def get_output_folder_name(self, use_scene_name=False):
        if use_scene_name:
            return self.scene_name + "_frames"
        else:
            return "../input"

    @staticmethod
    def set_up_scene_timeframe():
        bpy.context.scene.frame_start = 0
        bpy.context.scene.frame_end = 719

    @staticmethod
    def set_up_sky():
        """
        Prepare sky for rendering
        :return:
        """
        world = bpy.context.scene.world
        world.color = (1.0, 1.0, 1.0)
        world.use_nodes = False

    @staticmethod
    def set_up_mist():
        """
        Prepare the mist for rendering a raycast depth pass
        :return:
        """
        mist_settings = bpy.context.scene.world.mist_settings
        mist_settings.use_mist = True
        mist_settings.intensity = 0.0
        mist_settings.depth = RGBDCameraEmulatorApplication.MIST_DEPTH_M
        mist_settings.start = 0.0
        mist_settings.height = 0.0
        mist_settings.falloff = 'LINEAR'

    @staticmethod
    def set_up_light_helper():
        light = bpy.context.active_object.data
        light.energy = 10.0
        light.specular_factor = 1.0
        light.color = (1.0, 1.0, 1.0)
        light.falloff_type = 'INVERSE_SQUARE'
        light.cutoff_distance = 25.0
        light.use_shadow = False

    def set_up_lights(self):
        scene = bpy.context.scene

        # light 1
        scene.cursor.location = (1.042924404144287, 1.1423708200454712, 1.6510509252548218)
        bpy.ops.object.light_add(type='POINT')
        self.set_up_light_helper()
        lamp = bpy.context.active_object.data
        lamp.color = (1.0, 1.0, 1.0)
        lamp.energy = 10

        # light 2
        scene.cursor.location = (-2.0469818115234375, -2.4960832595825195, 0.08023321628570557)
        bpy.ops.object.light_add(type='POINT')
        self.set_up_light_helper()
        lamp = bpy.context.active_object.data
        lamp.color = (0.405, 0.604, 1.0)
        lamp.energy = 1.0

    def set_up_camera(self):
        scene = bpy.context.scene

        if self.args.make_camera_and_animate:
            # make the actual camera
            scene.cursor.location = (1.0, 0.0, 0.0)
            bpy.ops.object.camera_add()
            camera_object = bpy.context.active_object
            camera_object.rotation_euler = (pi / 2, 0.0, pi / 2)
            scene.camera = camera_object
        else:
            if scene.camera is None:
                raise ValueError("Cannot find active camera in scene. Either make a camera in the scene and animate "
                                 "as you see fit, or use the -mc argument to auto-generate a camera moving around "
                                 "the center of the scene.")
            camera_object = scene.camera

        # TODO: make multiple sensor support, current support is for Asus Xtion Pro only
        # configure it
        camera_data = camera_object.data
        camera_data.type = 'PERSP'
        camera_data.sensor_fit = 'HORIZONTAL'
        camera_data.angle = 1.0210176706314087
        camera_data.clip_start = 0.0000000001
        camera_data.clip_end = RGBDCameraEmulatorApplication.MIST_DEPTH_M
        camera_data.show_limits = False
        camera_data.show_mist = False
        camera_data.show_sensor = True
        camera_data.sensor_width = 32.0
        camera_data.dof.aperture_fstop = 128
        camera_data.display_size = 2.84

        if self.args.make_camera_and_animate:
            self.animate_camera(camera_object)

    @staticmethod
    def animate_camera(camera_object):
        # make a pivot empty for the camera at the center of the coordinate system
        scene = bpy.context.scene
        scene.cursor.location = (0.0, 0.0, 0.0)
        bpy.ops.object.empty_add(type='PLAIN_AXES')
        empty_object = bpy.context.active_object
        empty_object.name = "CameraController"

        # parent the cam to the empty
        camera_object.parent = empty_object

        # animate the camera by rotating the empty
        bpy.context.scene.frame_set(0)
        empty_object.keyframe_insert(data_path="rotation_euler", index=2)
        empty_object.keyframe_insert(data_path="rotation_euler", index=1)
        for frame in range(179, 720, 180):
            bpy.context.scene.frame_set(frame)
            # keyframe rotation to 0 around Y axis at quarter-turns
            empty_object.keyframe_insert(data_path="rotation_euler", index=1)

        y_up = True
        # around Y-axis
        for frame in range(89, 720, 180):
            bpy.context.scene.frame_set(frame)
            if y_up:
                empty_object.rotation_euler[1] = pi / 2
            else:
                empty_object.rotation_euler[1] = - pi / 2
            empty_object.keyframe_insert(data_path="rotation_euler", index=1)
            y_up = not y_up

        bpy.context.scene.frame_set(719)
        empty_object.rotation_euler[2] = pi * 2  # 360 degrees around z-axis
        empty_object.keyframe_insert(data_path="rotation_euler", index=2)

        # set interpolation of Z-axis rotation to linear
        empty_object.animation_data.action.fcurves[0].keyframe_points[0].interpolation = 'LINEAR'
        empty_object.animation_data.action.fcurves[0].keyframe_points[1].interpolation = 'LINEAR'
        # set the keyframe endpoints
        empty_object.animation_data.action.fcurves[1].keyframe_points[0].handle_right = (19, -.17)
        empty_object.animation_data.action.fcurves[1].keyframe_points[8].handle_left = (700, .17)

    @staticmethod
    def set_up_object_materials():
        # disable use of nodes on materials
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                for material_slot in obj.material_slots:
                    material_slot.material.use_nodes = False

    @staticmethod
    def set_up_cycles():
        scene = bpy.context.scene
        render_settings = scene.render
        render_settings.engine = 'CYCLES'
        scene.cycles.feature_set = 'SUPPORTED'
        scene.cycles.device = 'GPU'

        user_preferences = bpy.context.preferences
        cycles_preferences = user_preferences.addons['cycles'].preferences

        # Attempt to set GPU device types if available
        for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
            try:
                cycles_preferences.compute_device_type = compute_device_type
                break
            except TypeError:
                pass

        # Enable all CPU and GPU devices
        for device in cycles_preferences.devices:
            device.use = True

    @staticmethod
    def set_up_render_settings():
        scene = bpy.context.scene

        # correct render settings
        render_settings = scene.render
        render_settings.engine = 'BLENDER_EEVEE'

        scene.use_nodes = True

        # clear out unwanted render passes
        layer = scene.view_layers[0]
        layer.use_pass_normal = False
        layer.use_pass_vector = False
        layer.use_pass_uv = False
        layer.use_pass_z = False
        layer.use_pass_object_index = False
        layer.use_pass_material_index = False
        layer.use_pass_diffuse_color = False
        layer.use_pass_diffuse_direct = False
        layer.use_pass_diffuse_indirect = False
        layer.use_pass_glossy_color = False
        layer.use_pass_glossy_direct = False
        layer.use_pass_glossy_indirect = False
        layer.use_pass_subsurface_color = False
        layer.use_pass_subsurface_direct = False
        layer.use_pass_subsurface_indirect = False
        layer.use_pass_transmission_color = False
        layer.use_pass_transmission_direct = False
        layer.use_pass_transmission_indirect = False
        layer.use_pass_shadow = False
        layer.use_pass_emit = False
        layer.use_pass_ambient_occlusion = False
        layer.use_pass_environment = False
        layer.use_pass_combined = False

    def set_up_scene(self):
        """
        Set up the blender scene for all operations
        :return:
        """
        if self.args.make_camera_and_animate:
            # set up timeline bounds
            self.set_up_scene_timeframe()

        self.set_up_sky()
        self.set_up_mist()
        self.set_up_lights()
        self.set_up_camera()
        self.set_up_render_settings()
        self.set_up_object_materials()

    @staticmethod
    def rename_frames(folder, extension, new_filename_pattern):
        """
        Rename the frames at the specified folder, assuming they all have format <frame_number>.<extension>,
        using the given filename pattern
        :param extension: extension of the frame files
        :param folder: folder where to perform the operation
        :param new_filename_pattern: a string finelame pattern, which should include a single number format
        string (i.e. "{:03d}") inside somewhere
        :return:
        """
        for path in glob.iglob(os.path.join(folder, "*" + extension)):
            name, ext = os.path.splitext(os.path.basename(path))
            number = int(name)

            os.rename(path,
                      os.path.join(folder, new_filename_pattern.format(number) + ext))

    @staticmethod
    def check_folder_helper(task_name, folder):
        """
        Checks whether the specified folder exists, creates it if it doesn't.
        If there is a file or other non-folder entry at the folder path, returns False
        :param task_name: Name of the task we're creating this folder for
        :param folder: path of the desired folder
        :return: False if the folder doesn't exist or couldn't be created.
        """

        if not os.path.exists(folder):
            os.makedirs(folder)
        else:
            if not os.path.isdir(folder):
                print("The path %s does not represent a folder (perhaps, a file?). " +
                      "Cannot %s, skipping this step.".format(folder, task_name))
                return False
        return True

    def generate_camera_poses(self, scene_folder, save_numpy_originals=False, save_numpy_converted=False):
        """
        Outputs poses (4x4 translation & rotation matrices relative to world) of the active camera
        in the currently-open Blender file for each time frame.
        :type scene_folder: str
        :param scene_folder: folder where to put the <scene_name>_camera_poses with the generated poses.
        :type save_numpy_originals: bool
        :param save_numpy_originals: Whether to save an npz archive containing the original camera poses
        :type save_numpy_converted: bool
        :param save_numpy_converted: Whether to save an npz archive containing the poses after the axes
        & coordinate system conversion (before the transpose)
        :rtype bool
        :return: True on operation success, False otherwise
        """
        # === prepare the folder where we're going to be saving the data ===
        scene_name = os.path.basename(scene_folder)
        pose_folder = os.path.join(scene_folder, scene_name + "_camera_poses")

        if not self.check_folder_helper("generate camera poses", pose_folder):
            return False

        # === extract the actual camera pose matrices in numpy array format ===
        cam = bpy.context.scene.camera
        cam_pose_list = []
        for i_frame in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
            bpy.context.scene.frame_set(i_frame)
            cam_pose_list.append(np.array(cam.matrix_world))
        poses = np.array(cam_pose_list)

        # === convert to Kangaroo-compatible eigen format (with axes adjustments) ===
        rotation_mat = np.array([[1.0, 0.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0, 0.0],  # flip Y
                                 [0.0, 0.0, -1.0, 0.0],  # flip Z
                                 [0.0, 0.0, 0., 1.0]])
        first_pose_inv = np.linalg.inv(poses[0])
        converted_poses = []

        count = 0
        for pose in poses:
            # first camera pose should be all-zeros, so adjust the coordinate system
            pose = first_pose_inv.dot(pose)
            # apply the axes transformation
            pose = rotation_mat.dot(pose).dot(rotation_mat)

            converted_poses.append(pose)
            # transpose, because Eigen is column-major by default
            transposed = pose.transpose()
            size = np.array([4, 4], dtype=np.uint64)

            pose_file_name = os.path.join(pose_folder, '{0:04d}.pose'.format(count))
            file_out = open(pose_file_name, "wb")
            file_out.write(struct.pack('L' * 2, *size))
            for i in range(0, 4):
                file_out.write(struct.pack('d' * 4, *transposed[i]))
            file_out.close()
            count += 1

        # save output in numpy format if requested
        if save_numpy_converted:
            np.savez_compressed("converted_poses.npz", poses=np.array(converted_poses))
        if save_numpy_originals:
            np.savez_compressed("/home/algomorph/Factory/priors/test_data/coffee_cup/camera_poses.npz", poses=poses)
        return True

    @staticmethod
    def set_render_settings_helper(render_settings):
        # TODO: provide support for multiple sensors (current support is for Asus Xtion Pro emulation only)
        render_settings.resolution_x = 640
        render_settings.resolution_y = 480
        render_settings.resolution_percentage = 100

    def generate_rgb_frames(self, scene_folder, frames_folder):
        """
        Renders each frame of the scene with the currently-active camera to an RGB 8-bit png using the
        composite (color) pass and not using the mist pass
        :param frames_folder: folder where to combine the final rgb & depth renders
        :type scene_folder: str
        :param scene_folder: folder where to put the rgb_renders folder with the generated rgb renders
        :rtype bool
        :return: True on operation success, False otherwise
        """
        # reset to turn mist off, just in case
        mist_settings = bpy.context.scene.world.mist_settings
        mist_settings.use_mist = False

        scene = bpy.context.scene  # type: bpy.types.Scene
        render_settings = scene.render  # type: bpy.types.RenderSettings

        rgb_folder = os.path.join(scene_folder, "rgb_renders")

        if not self.check_folder_helper("render rgb images / composite pass", rgb_folder):
            return False

        # === set up the image format ===
        render_settings.filepath = rgb_folder + os.path.sep  # tell blender where to render the files
        scene.display.render_aa = 'OFF'

        render_settings.image_settings.file_format = 'PNG'
        render_settings.image_settings.color_mode = 'RGB'
        render_settings.image_settings.color_depth = '8'
        self.set_render_settings_helper(render_settings)

        world = scene.world  # type: bpy.types.World
        world.color = (0.0, 0.0, 0.0)

        # === adjust the render layers to not use mist/depth but to render the combined pass ===
        render_layer = scene.view_layers[0]  # type: bpy.types.RenderLayers
        render_layer.use_pass_combined = True
        render_layer.use_pass_mist = False
        render_layer.use_pass_z = False

        # === adjust the compositor to use only "Image" output from render layers, not the "Mist"
        node_tree = bpy.context.scene.node_tree
        render_layers_node = node_tree.nodes.get("Render Layers")
        composite_node = node_tree.nodes.get("Composite")
        links = node_tree.links
        links.clear()
        links.new(render_layers_node.outputs["Image"], composite_node.inputs["Image"])

        bops.render.render(animation=True)

        # === rename & move the files ===
        self.rename_frames(rgb_folder, ".png", "color_{:05d}")
        filelist = glob.glob(os.path.join(rgb_folder, "*.png"))
        for path in filelist:
            name = os.path.basename(path)
            shutil.copy(path, os.path.join(frames_folder, name))
        shutil.rmtree(rgb_folder)  # cleanup

        return True

    @staticmethod
    def precompute_cosines(image, focal_length_ratio):
        center_x = image.shape[1] / 2 - .5
        center_y = image.shape[0] / 2 - .5
        polar_distances = np.zeros(image.shape, dtype=float)
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                x_offset = x - center_x
                y_offset = y - center_y
                dist_from_center = x_offset * x_offset + y_offset * y_offset
                polar_distances[y, x] = dist_from_center
        polar_distances = np.sqrt(polar_distances)
        return np.cos(np.arctan(polar_distances / focal_length_ratio))

    def generate_depth_frames(self, scene_folder, frames_folder):
        """
        Renders each frame of the scene with the currently-active camera using the mist pass only into
        OpenEXR floating-point format
        :param frames_folder: folder where to copy the final output
        :type scene_folder: str
        :param scene_folder: folder where to put the depth_renders folder with the generated deopth renders
        :rtype bool
        :return: True on operation success, False otherwise
        """
        # === prepare the folder where we're going to be saving the renders ===
        depth_folder = os.path.join(scene_folder, "depth_renders")

        if not self.check_folder_helper("render depth images / mist pass", depth_folder):
            return False

        # reset to turn mist on, just in case
        mist_settings = bpy.context.scene.world.mist_settings
        mist_settings.use_mist = True

        scene = bpy.context.scene
        scene.display.render_aa = 'OFF'

        render_settings = scene.render  # type: bpy.types.RenderSettings
        render_settings.filepath = depth_folder + os.path.sep  # tell blender where to render the files
        render_settings.image_settings.file_format = 'OPEN_EXR'
        render_settings.image_settings.color_mode = 'BW'
        render_settings.image_settings.color_depth = '32'
        render_settings.image_settings.exr_codec = 'ZIP'
        self.set_render_settings_helper(render_settings)

        world = scene.world  # type: bpy.types.World
        world.color = (1.0, 1.0, 1.0)  # sets mist color to white

        # === adjust the render layers to use mist/depth, not the combined pass ===
        render_layer = scene.view_layers[0]  # type: bpy.types.RenderLayers
        render_layer.use_pass_combined = False
        render_layer.use_pass_mist = True
        render_layer.use_pass_z = False

        # === adjust the node tree to use the Mist output
        node_tree = bpy.context.scene.node_tree
        render_layers_node = node_tree.nodes.get("Render Layers")
        composite_node = node_tree.nodes.get("Composite")
        links = node_tree.links
        links.clear()
        links.new(render_layers_node.outputs["Mist"], composite_node.inputs["Image"])

        bops.render.render(animation=True)

        # ==========|| Apply the ray-to-depth conversion ||==================
        depth_render_filenames = glob.glob(os.path.join(depth_folder, "*.exr"))
        depth_render_filenames.sort()

        # assumes the camera has already been set up and it's in horizontal mode
        # == precompute cosines of angle to each pixel from optical axis
        scene = bpy.context.scene
        camd = scene.camera.data
        # otherwise known as f_{x} in the intrinsic matrix:
        focal_length_ratio = render_settings.resolution_x / camd.sensor_width * camd.lens
        cosines = self.precompute_cosines(cv2.imread(depth_render_filenames[0],
                                                     cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)[:, :, 0],
                                          focal_length_ratio)
        i_frame = 0
        for filename in depth_render_filenames:
            image = cv2.imread(filename, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH).astype(np.float)[:, :, 0]

            image[image == 1.0] = 0.0

            image *= cosines  # raycast-to-depth
            image *= (RGBDCameraEmulatorApplication.MIST_DEPTH_M * 1000.0)  # rescale to OpenNI/Xtion format

            cv2.imwrite(os.path.join(frames_folder, 'depth_{0:05d}.png'.format(i_frame)),
                        image.astype(np.uint16))
            i_frame += 1
            os.remove(filename)  # clean up

        os.removedirs(depth_folder)  # clean up

        return True

    def run(self):
        # tune based on preferences
        # TODO: make input args
        store_depth_and_rgb_frames_in_same_folder = True
        scene_name_up_two_levels = False

        blender_filename = os.path.basename(self.args.input)
        if not blender_filename.endswith(".blend"):
            print("Warning: the blender filename does not end with .blend. " +
                  "Only valid blender files with only geometry object(s) in the first layer are supported.")
        scene_folder = os.path.dirname(self.args.input)
        bpy.ops.wm.open_mainfile(filepath=self.args.input)
        self.set_up_scene()

        if scene_name_up_two_levels:
            scene_name = os.path.basename(os.path.dirname(scene_folder))
        else:
            # assume 1 level
            scene_name = os.path.basename(scene_folder)
        self.scene_name = scene_name

        # === prepare the folder where we're going to be saving the renders ===
        if store_depth_and_rgb_frames_in_same_folder:
            frames_folder = os.path.join(scene_folder, self.get_output_folder_name(True))
        else:
            frames_folder = os.path.join(scene_folder, scene_name + "_color")
        color_frames_folder = frames_folder
        if not self.check_folder_helper("render rgb images / composite pass", frames_folder):
            return 1

        if self.args.rgb:
            self.generate_rgb_frames(scene_folder, frames_folder)
        if self.args.depth:
            if not store_depth_and_rgb_frames_in_same_folder:
                frames_folder = os.path.join(scene_folder, scene_name + "_depth")
            self.generate_depth_frames(scene_folder, frames_folder)
        if self.args.poses:
            self.generate_camera_poses(scene_folder)
        if self.args.process_segmentation:
            if not self.args.rgb and not os.path.isdir(color_frames_folder):
                raise ValueError("Turn on rgb frame rendering to also generate label segments and contours.")
            labels_folder = os.path.join(scene_folder, scene_name + "_labels")
            if not self.check_folder_helper("render label segmentation images / label pass", labels_folder):
                return 1
            contours_folder = os.path.join(scene_folder, scene_name + "_contours")
            if not self.check_folder_helper("render label segmentation images / label pass", contours_folder):
                return 1
            color_frame_regex = re.compile(r'color_(\d\d\d\d\d).png')

            frame_list = list(filter(lambda folder: color_frame_regex.match(folder),
                                     os.listdir(color_frames_folder)))
            frame_list.sort()
            label_mapping = {}
            for file_name in frame_list:
                file_path = os.path.join(color_frames_folder, file_name)
                color_frame = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
                frame_number = int(color_frame_regex.match(file_name).group(1))
                labels_frame, label_mapping = make_labeling_consistent(color_frame, label_mapping)
                contours_frame = draw_contours(color_frame, 0, 1, False) * 255
                cv2.imwrite(os.path.join(contours_folder, "contours_{:05d}.png".format(frame_number)), contours_frame)
                cv2.imwrite(os.path.join(labels_folder, "labels_{:05d}.png".format(frame_number)), labels_frame)

        if self.args.save_copy:
            blend_file_path = os.path.join(scene_folder, os.path.splitext(blender_filename)[0] + "_postprep.blend")
            print(blend_file_path)
            bpy.ops.wm.save_as_mainfile(filepath=blend_file_path)


EXIT_CODE_FAILURE = 1
EXIT_CODE_SUCCESS = 0


def main():
    parser = argparse.ArgumentParser("A script that renders a blender file using the Mist pass, " +
                                     "converts that to OpenNi-compatible depth frame format, renders" +
                                     " the RGB / composite pass, and outputs the camera poses, to prep" +
                                     " the mesh for Fusion-based TSDF decimation or any kind of depth processing.")

    parser.add_argument("-i", "--input", help="The input blender file.",
                        default="../synthetic input/plane/plane.blend")
    parser.add_argument("-s", "--save_copy",
                        help="Save a copy of the file after setup & operations into " +
                             "<original_input_path>/<original_input_filename>_postprep.blend.",
                        default=False, action='store_true')
    parser.add_argument("-nd", "--no-depth", dest="depth", action='store_false', default=True)
    parser.add_argument("-nr", "--no-rgb", dest="rgb", action='store_false', default=True)
    parser.add_argument("-np", "--no-poses", dest="poses", action='store_false', default=True)
    parser.add_argument("-mc", "--make_camera_and_animate", action='store_true', default=False,
                        help="Use for static objects. Instead of using the first camera in the scene, makes one.")

    parser.add_argument("-seg", "--process_segmentation", action='store_true', default=False,
                        help="Assumes the scene is colored with shadeless materials (!): process segmentations separately,"
                             " producing a contour image set and a set of greyscale images with segment pixels colored by"
                             " increasing values, i.e. 0, 1, 2, 3...")
    exit_code = EXIT_CODE_SUCCESS
    args = parser.parse_args()
    app = RGBDCameraEmulatorApplication(args)
    app.run()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
