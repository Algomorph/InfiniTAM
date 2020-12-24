#!/usr/bin/python3

# Linux^
# To use in bash under Windows, replace first line with the following, edit as appropriate, and prepend '!':
# /c/Users/algom/AppData/Local/Programs/Python/Python37/python

###############################################################################
# Example script to use GIMP via python scripting to prepare files for manual #
# mask editing and export final masks afterward as 'png' frames.              #
#
# Usage:
# <python exec or nothing if in bash> <gimp_batch_script filename.py> <batch function name>
#
# Batch functions are defined under 'BATCH FUNCTIONS' in the code below.
#
###############################################################################
import os

def prep_mask_for_editing(color_frame_path, mask_frame_path, output_directory):
    color_image = pdb.gimp_file_load(color_frame_path, color_frame_path)
    mask_image = pdb.gimp_file_load(mask_frame_path, mask_frame_path)
    buffer_index = pdb.gimp_edit_copy(mask_image.layers[0])
    floating_layer = pdb.gimp_edit_paste(color_image.layers[0], buffer_index)
    floating_layer.opacity = 50.0
    pdb.gimp_floating_sel_to_layer(floating_layer)
    frame_index_string = os.path.splitext(os.path.basename(color_frame_path))[0][-6:]
    output_path = os.path.join(output_directory, "mask_prep_" + frame_index_string + ".xcf")
    pdb.gimp_xcf_save(0, color_image, color_image.active_drawable, output_path, output_path)
    pdb.gimp_image_delete(color_image)
    pdb.gimp_image_delete(mask_image)


def save_prepped_mask(mask_prep_path, mask_frame_path, output_directory):
    prep_image = pdb.gimp_file_load(mask_prep_path, mask_prep_path)
    mask_image = pdb.gimp_file_load(mask_frame_path, mask_frame_path)
    buffer_index = pdb.gimp_edit_copy(prep_image.layers[0])
    floating_layer = pdb.gimp_edit_paste(mask_image.layers[0], buffer_index)
    pdb.gimp_floating_sel_anchor(floating_layer)
    frame_index_string = os.path.splitext(os.path.basename(mask_prep_path))[0][-6:]
    output_path = os.path.join(output_directory, "imbs_mask_" + frame_index_string + ".png")
    pdb.file_png_save(mask_image, mask_image.active_drawable, output_path, output_path, 0, 9, 1, 0, 0, 1, 1)


def clear_alpha_png_image(path):
    mask_image = pdb.gimp_file_load(path, path)
    pdb.gimp_image_flatten(mask_image)
    pdb.file_png_save(mask_image, mask_image.active_drawable, path, path, 0, 9, 1, 0, 0, 1, 1)


def get_frames(directory, prefix):
    frame_paths = []
    for filename in os.listdir(directory):
        if filename.startswith(prefix):
            frame_paths.append(os.path.join(directory, filename))
    frame_paths.sort()
    return frame_paths


#### BATCH FUNCTIONS ######

def mask_edit_prep(color_frame_directory="E:/Reconstruction/real_data/snoopy/color",
                   raw_mask_directory="E:/Reconstruction/real_data/snoopy/imbs_masks",
                   output_directory="E:/Reconstruction/real_data/snoopy/mask_edit_prep"):
    color_frame_paths = get_frames(color_frame_directory, "color_")
    mask_frame_paths = get_frames(raw_mask_directory, "imbs_mask_")
    for color_frame_path, mask_frame_path in zip(color_frame_paths, mask_frame_paths):
        prep_mask_for_editing(color_frame_path, mask_frame_path, output_directory)


def mask_edit_save(mask_prep_directory="E:/Reconstruction/real_data/snoopy/mask_edit_prep_686-715",
                   raw_mask_directory="E:/Reconstruction/real_data/snoopy/imbs_masks",
                   output_directory="E:/Reconstruction/real_data/snoopy/imbs_masks_edited"):
    mask_prep_paths = get_frames(mask_prep_directory, "mask_prep_")
    mask_frame_paths = get_frames(raw_mask_directory, "imbs_mask_")
    for mask_prep_path, mask_frame_path in zip(mask_prep_paths, mask_frame_paths):
        save_prepped_mask(mask_prep_path, mask_frame_path, output_directory)


def clear_alpha_masks(directory="/mnt/Data/Reconstruction/real_data/snoopy/masks"):
    mask_paths = get_frames(directory, "mask_")
    for mask_path in mask_paths:
        clear_alpha_png_image(mask_path)


# GIMP auto-execution stub
if __name__ == "__main__":
    import os, sys, subprocess

    if len(sys.argv) < 2:
        print("you must specify a function to execute!")
        sys.exit(-1)
    scrdir = os.path.dirname(os.path.realpath(__file__))
    scrname = os.path.splitext(os.path.basename(__file__))[0]
    shcode = "import sys;sys.path.insert(0, '" + scrdir + "');import " + scrname + ";" + scrname + "." + sys.argv[1] + str(tuple(sys.argv[2:]))
    shcode = "gimp-console -idf --batch-interpreter python-fu-eval -b \"" + shcode + "\" -b \"pdb.gimp_quit(1)\""
    sys.exit(subprocess.call(shcode, shell=True))
else:
    from gimpfu import *
