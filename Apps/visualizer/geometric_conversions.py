class VoxelVolumeParameters:
    BLOCK_WIDTH_VOXELS = 8
    VOXEL_SIZE = 0.004  # meters
    BLOCK_SIZE = BLOCK_WIDTH_VOXELS * VOXEL_SIZE
    BLOCK_OFFSET = (VOXEL_SIZE / 2) + (BLOCK_SIZE / 2)


def convert_block_to_metric(input_in_block_coordinates):
    return input_in_block_coordinates * VoxelVolumeParameters.BLOCK_SIZE + VoxelVolumeParameters.BLOCK_OFFSET
