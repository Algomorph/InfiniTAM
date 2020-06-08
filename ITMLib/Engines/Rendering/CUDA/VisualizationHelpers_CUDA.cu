// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "VisualizationHelpers_CUDA.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../../ORUtils/PlatformIndependedParallelSum.h"

using namespace ITMLib;
__global__ void ITMLib::checkProjectAndSplitBlocks_device(const HashEntry *hashEntries, int noHashEntries,
                                                          const Matrix4f pose_M, const Vector4f intrinsics, const Vector2i imgSize, float voxelSize, RenderingBlock *renderingBlocks,
                                                          uint *noTotalBlocks)
{
	int targetIdx = threadIdx.x + blockDim.x * blockIdx.x;
	if (targetIdx >= noHashEntries) return;

	const HashEntry & hashEntry = hashEntries[targetIdx];

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool validProjection = false;
	if (hashEntry.ptr >= 0) validProjection = ProjectSingleBlock(hashEntry.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);

	Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / rendering_block_size_x),
		ceilf((float)(lowerRight.y - upperLeft.y + 1) / rendering_block_size_y));
	size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!validProjection) requiredNumBlocks = 0;

	int out_offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<uint>(requiredNumBlocks, noTotalBlocks);
	if (requiredNumBlocks == 0) return;
	if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__global__ void ITMLib::fillBlocks_device(uint noTotalBlocks, const RenderingBlock *renderingBlocks,
	Vector2i imgSize, Vector2f *minmaxData)
{
	int x = threadIdx.x;
	int y = threadIdx.y;
	int block = blockIdx.x * 4 + blockIdx.y;
	if (block >= noTotalBlocks) return;

	const RenderingBlock & b(renderingBlocks[block]);
	int xpos = b.upper_left.x + x;
	if (xpos > b.lower_right.x) return;
	int ypos = b.upper_left.y + y;
	if (ypos > b.lower_right.y) return;

	Vector2f & pixel(minmaxData[xpos + ypos*imgSize.x]);
	atomicMin(&pixel.x, b.z_range.x); atomicMax(&pixel.y, b.z_range.y);
}