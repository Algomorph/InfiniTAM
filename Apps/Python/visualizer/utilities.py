import os


def get_frame_output_path(output_path, i_frame):
    return os.path.join(output_path, "Frame_{:03}".format(i_frame))


def get_output_frame_count(output_path):
    contents = os.listdir(output_path)
    count = 0
    for item in contents:
        full_path = os.path.join(output_path, item)
        if os.path.isdir(full_path) and item.startswith("Frame_"):
            count += 1
    return count
