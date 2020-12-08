// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "VisualizationHelpers_CUDA.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../../ORUtils/PlatformIndependentParallelSum.h"

using namespace ITMLib;

__global__ void ITMLib::checkProjectAndSplitBlocks_device(const HashEntry* hash_entries, int noHashEntries,
                                                          const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize,
                                                          RenderingBlock* renderingBlocks,
                                                          uint* noTotalBlocks) {
	int targetIdx = threadIdx.x + blockDim.x * blockIdx.x;
	if (targetIdx >= noHashEntries) return;

	const HashEntry& hashEntry = hash_entries[targetIdx];

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool projection_is_valid = false;
	if (hashEntry.ptr >= 0)
		projection_is_valid = ProjectSingleBlock(hashEntry.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);

	Vector2i requiredRenderingBlocks(ceilf((float) (lowerRight.x - upperLeft.x + 1) / rendering_block_size_x),
	                                 ceilf((float) (lowerRight.y - upperLeft.y + 1) / rendering_block_size_y));
	size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!projection_is_valid) requiredNumBlocks = 0;

	int out_offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<uint>(requiredNumBlocks, noTotalBlocks);
	if (requiredNumBlocks == 0) return;
	if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__global__ void ITMLib::fillBlocks_device(uint block_count, const RenderingBlock* rendering_blocks,
                                          Vector2i image_size, Vector2f* pixel_ray_data_ranges) {
	int x = threadIdx.x;
	int y = threadIdx.y;
	int block = blockIdx.x * 4 + blockIdx.y;
	if (block >= block_count) return;

	const RenderingBlock& b(rendering_blocks[block]);
	int xpos = b.upper_left.x + x;
	if (xpos > b.lower_right.x) return;
	int ypos = b.upper_left.y + y;
	if (ypos > b.lower_right.y) return;

	Vector2f& pixel(pixel_ray_data_ranges[xpos + ypos * image_size.x]);
	atomicMin(&pixel.x, b.z_range.x);
	atomicMax(&pixel.y, b.z_range.y);
}