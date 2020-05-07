import os
import gzip
import numpy as np


def load_inverse_matrices(output_folder):
    path = os.path.join(output_folder, "camera_matrices.dat")
    file = gzip.open(path, "rb")
    inverse_camera_matrices = []
    while file.readable():
        buffer = file.read(size=64)
        if not buffer:
            break
        inverse_camera_matrix = np.linalg.inv(np.resize(np.frombuffer(buffer, dtype=np.float32), (4, 4)).T)
        inverse_camera_matrices.append(inverse_camera_matrix)
    print("Read camera matrices for", len(inverse_camera_matrices), "frames.")
    return inverse_camera_matrices
