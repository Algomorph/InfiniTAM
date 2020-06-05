// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../../Objects/Views/View.h"

#include "../Shared/RenderingEngine_Shared.h"
#include "../../Reconstruction/Shared/SceneReconstructionEngine_Shared.h"
#include "../../../Utils/CUDAUtils.h"
#include "../../../../ORUtils/PlatformIndependedParallelSum.h"

namespace ITMLib {
// declaration of device functions

__global__ void buildCompleteVisibleList_device(
		const HashEntry* hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/ int hashBlockCount,
		int* visibleBlockHashCodes, int* visibleBlockCount, HashBlockVisibility* blockVisibilityTypes, Matrix4f M,
		Vector4f projParams, Vector2i imgSize, float voxelSize);

__global__ void
countVisibleBlocks_device(const int* visibleEntryIDs, int visibleBlockCount, const HashEntry* hashTable,
                          uint* noBlocks, int minBlockId, int maxBlockId);

__global__ void
projectAndSplitBlocks_device(const HashEntry* hash_table, const int* hash_codes, int block_count,
                             const Matrix4f depth_camera_pose, const Vector4f depth_camera_projection_parameters, const Vector2i depth_image_size, float voxel_size,
                             RenderingBlock* rendering_blocks, uint* total_rendering_block_count);

__global__ void checkProjectAndSplitBlocks_device(const HashEntry* hashEntries, int noHashEntries,
                                                  const Matrix4f pose_M, const Vector4f intrinsics,
                                                  const Vector2i imgSize, float voxelSize,
                                                  RenderingBlock* renderingBlocks,
                                                  uint* noTotalBlocks);

__global__ void fillBlocks_device(uint noTotalBlocks, const RenderingBlock* renderingBlocks,
                                  Vector2i imgSize, Vector2f* minmaxData);

__global__ void findMissingPoints_device(int* fwdProjMissingPoints, uint* noMissingPoints, const Vector2f* minmaximg,
                                         Vector4f* forwardProjection, float* currentDepth, Vector2i imgSize);

__global__ void
forwardProject_device(Vector4f* forwardProjection, const Vector4f* pointsRay, Vector2i imgSize, Matrix4f M,
                      Vector4f projParams, float voxelSize);

template<class TVoxel, class TIndex, bool modifyVisibleEntries>
__global__ void genericRaycast_device(Vector4f* out_ptsRay, HashBlockVisibility* blockVisibilityTypes, const TVoxel* voxelData,
                                      const typename TIndex::IndexData* voxelIndex, Vector2i depth_image_size, Matrix4f invM,
                                      Vector4f invProjParams,
                                      float oneOverVoxelSize, const Vector2f* minmaximg, float mu) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= depth_image_size.width || y >= depth_image_size.height) return;

	int locId = x + y * depth_image_size.width;
	int locId2 =
			(int) floor((float) x / ray_depth_image_subsampling_factor) + (int) floor((float) y / ray_depth_image_subsampling_factor) * depth_image_size.width;

	CastRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], blockVisibilityTypes, x, y, voxelData, voxelIndex,
	                                              invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
}

template<class TVoxel, class TIndex, bool modifyVisibleEntries>
__global__ void
genericRaycastMissingPoints_device(Vector4f* forwardProjection, HashBlockVisibility* blockVisibilityTypes, const TVoxel* voxelData,
                                   const typename TIndex::IndexData* voxelIndex, Vector2i imgSize, Matrix4f invM,
                                   Vector4f invProjParams, float oneOverVoxelSize,
                                   int* fwdProjMissingPoints, int noMissingPoints, const Vector2f* minmaximg,
                                   float mu) {
	int pointId = threadIdx.x + blockIdx.x * blockDim.x;

	if (pointId >= noMissingPoints) return;

	int locId = fwdProjMissingPoints[pointId];
	int y = locId / imgSize.x, x = locId - y * imgSize.x;
	int locId2 =
			(int) floor((float) x / ray_depth_image_subsampling_factor) + (int) floor((float) y / ray_depth_image_subsampling_factor) * imgSize.x;

	CastRay<TVoxel, TIndex, modifyVisibleEntries>(forwardProjection[locId], blockVisibilityTypes, x, y, voxelData,
	                                              voxelIndex, invM, invProjParams, oneOverVoxelSize, mu,
	                                              minmaximg[locId2]);
}

template<bool flipNormals>
__global__ void renderICP_device(Vector4f* pointsMap, Vector4f* normalsMap, const Vector4f* pointsRay,
                                 float voxelSize, Vector2i imgSize, Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelICP<true, flipNormals>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

template<bool flipNormals>
__global__ void
renderGrey_ImageNormals_device(Vector4u* outRendering, const Vector4f* pointsRay, float voxelSize, Vector2i imgSize,
                               Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelGrey_ImageNormals<true, flipNormals>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
}

template<bool flipNormals>
__global__ void
renderNormals_ImageNormals_device(Vector4u* outRendering, const Vector4f* ptsRay, Vector2i imgSize, float voxelSize,
                                  Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelNormals_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
}

template<bool flipNormals>
__global__ void
renderConfidence_ImageNormals_device(Vector4u* outRendering, const Vector4f* ptsRay, Vector2i imgSize, float voxelSize,
                                     Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	processPixelConfidence_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
}

template<class TVoxel, class TIndex>
__global__ void renderGrey_device(Vector4u* outRendering, const Vector4f* ptsRay, const TVoxel* voxelData,
                                  const typename TIndex::IndexData* voxelIndex, Vector2i imgSize,
                                  Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
	                                 lightSource);
}

template<class TVoxel, class TIndex>
__global__ void renderColourFromNormal_device(Vector4u* outRendering, const Vector4f* ptsRay, const TVoxel* voxelData,
                                              const typename TIndex::IndexData* voxelIndex, Vector2i imgSize,
                                              Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
	                                   lightSource);
}

template<class TVoxel, class TIndex>
__global__ void
renderColourFromConfidence_device(Vector4u* outRendering, const Vector4f* ptsRay, const TVoxel* voxelData,
                                  const typename TIndex::IndexData* voxelIndex, Vector2i imgSize,
                                  Vector3f lightSource) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex, lightSource);
}

template<class TVoxel, class TIndex>
__global__ void
renderPointCloud_device(/*Vector4u *outRendering, */Vector4f* locations, Vector4f* colours, uint* noTotalPoints,
                                                    const Vector4f* ptsRay, const TVoxel* voxelData,
                                                    const typename TIndex::IndexData* voxelIndex, bool skipPoints,
                                                    float voxelSize, Vector2i imgSize, Vector3f lightSource) {
	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	bool foundPoint = false;
	Vector3f point(0.0f);

	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x < imgSize.x && y < imgSize.y) {
		int locId = x + y * imgSize.x;
		Vector3f outNormal;
		float angle;
		Vector4f pointRay;

		pointRay = ptsRay[locId];
		point = pointRay.toVector3();
		foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint) shouldPrefix = true;
	}

	__syncthreads();

	if (shouldPrefix) {
		int offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add2D<uint>(foundPoint, noTotalPoints);

		if (offset != -1) {
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex,
			                                                                                 point);
			if (tmp.w > 0.0f) {
				tmp.x /= tmp.w;
				tmp.y /= tmp.w;
				tmp.z /= tmp.w;
				tmp.w = 1.0f;
			}
			colours[offset] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize;
			pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize;
			pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;
		}
	}
}

template<class TVoxel, class TIndex>
__global__ void renderColour_device(Vector4u* outRendering, const Vector4f* ptsRay, const TVoxel* voxelData,
                                    const typename TIndex::IndexData* voxelIndex, Vector2i imgSize) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
}
}
