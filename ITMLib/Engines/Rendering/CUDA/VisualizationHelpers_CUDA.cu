// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "VisualizationHelpers_CUDA.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../../ORUtils/PlatformIndependedParallelSum.h"

using namespace ITMLib;

//device implementations

__global__ void ITMLib::countVisibleBlocks_device(const int *visibleEntryIDs, int visibleBlockCount, const HashEntry *hashTable, uint *noBlocks, int minBlockId, int maxBlockId)
{
	int globalIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (globalIdx >= visibleBlockCount) return;

	int entryId = visibleEntryIDs[globalIdx];
	int blockId = hashTable[entryId].ptr;
	if ((blockId >= minBlockId) && (blockId <= maxBlockId)) atomicAdd(noBlocks, 1);
}

__global__ void ITMLib::buildCompleteVisibleList_device(const HashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int hashBlockCount,
                                                        int *visibleBlockHashCodes, int *visibleBlockCount, HashBlockVisibility* blockVisibilityTypes, Matrix4f M, Vector4f projParams, Vector2i imgSize, float voxelSize)
{
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > hashBlockCount - 1) return;

	__shared__ bool shouldPrefix;

	unsigned char hashVisibleType = 0; //blockVisibilityTypes[targetIdx];
	const HashEntry &hashEntry = hashTable[targetIdx];

	shouldPrefix = false;
	__syncthreads();

	if (hashEntry.ptr >= 0)
	{
		shouldPrefix = true;

		bool isVisible, isVisibleEnlarged;
		CheckVoxelHashBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize,
		                                     imgSize);

		hashVisibleType = isVisible;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<int>(hashVisibleType > 0, visibleBlockCount);
		if(targetIdx == 296903){
			printf("offset 296903: %d\n", offset);
		}
		if(targetIdx == 123480){
			printf("offset 123480: %d\n", offset);
		}
		if (offset != -1) visibleBlockHashCodes[offset] = targetIdx;
	}
}

__global__ void ITMLib::projectAndSplitBlocks_device(const HashEntry* hash_table, const int* hash_codes, int block_count,
                                                     const Matrix4f depth_camera_pose, const Vector4f depth_camera_projection_parameters,
                                                     const Vector2i depth_image_size, float voxel_size, RenderingBlock* rendering_blocks,
                                                     uint* total_rendering_block_count) {
	int in_offset = threadIdx.x + blockDim.x * blockIdx.x;

	const HashEntry& blockData(hash_table[hash_codes[in_offset]]);

	Vector2i upperLeft, lowerRight;
	Vector2f zRange;
	bool validProjection = false;
	if (in_offset < block_count)
		if (blockData.ptr >= 0)
			validProjection = ProjectSingleBlock(blockData.pos, depth_camera_pose, depth_camera_projection_parameters, depth_image_size, voxel_size,
			                                     upperLeft, lowerRight, zRange);

	Vector2i requiredRenderingBlocks(ceilf((float) (lowerRight.x - upperLeft.x + 1) / rendering_block_size_x),
	                                 ceilf((float) (lowerRight.y - upperLeft.y + 1) / rendering_block_size_y));

	size_t required_local_rendering_block_count = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
	if (!validProjection) required_local_rendering_block_count = 0;

	int out_offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<uint>(required_local_rendering_block_count, total_rendering_block_count);
	if (!validProjection) return;
	if ((out_offset == -1) || (out_offset + required_local_rendering_block_count > MAX_RENDERING_BLOCKS)) return;

	CreateRenderingBlocks(rendering_blocks, out_offset, upperLeft, lowerRight, zRange);
}

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

__global__ void ITMLib::findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
	Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

	Vector4f fwdPoint = forwardProjection[locId];
	Vector2f minmaxval = minmaximg[locId2];
	float depth = currentDepth[locId];

	bool hasPoint = false;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth > 0)) && (minmaxval.x < minmaxval.y))
		//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
	{
		shouldPrefix = true; hasPoint = true;
	}

	__syncthreads();

	if (shouldPrefix)
	{
		int offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add2D<uint>(hasPoint, noMissingPoints);
		if (offset != -1) fwdProjMissingPoints[offset] = locId;
	}
}

__global__ void ITMLib::forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize, Matrix4f M,
	Vector4f projParams, float voxelSize)
{
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;
	Vector4f pixel = pointsRay[locId];

	int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
	if (locId_new >= 0) forwardProjection[locId_new] = pixel;
}
