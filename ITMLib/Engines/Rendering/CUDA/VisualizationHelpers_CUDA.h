// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../../Objects/Views/View.h"

#include "../Shared/RenderingEngine_Shared.h"
#include "../../Reconstruction/Shared/SceneReconstructionEngine_Shared.h"
#include "../../../Utils/CUDA/CUDAUtils.h"
#include "../../../../ORUtils/PlatformIndependentParallelSum.h"

namespace ITMLib {
// declaration of device functions

__global__ void checkProjectAndSplitBlocks_device(const HashEntry* hash_entries, int noHashEntries,
                                                  const Matrix4f pose_M, const Vector4f intrinsics,
                                                  const Vector2i imgSize, float voxelSize,
                                                  RenderingBlock* renderingBlocks,
                                                  uint* noTotalBlocks);

__global__ void fillBlocks_device(uint block_count, const RenderingBlock* rendering_blocks,
                                  Vector2i image_size, Vector2f* pixel_ray_data_ranges);

template<class TVoxel, class TIndex, bool modifyVisibleEntries>
__global__ void genericRaycast_device(Vector4f* out_ptsRay, HashBlockVisibility* block_visibility_types, const TVoxel* voxels,
                                      const typename TIndex::IndexData* index_data, Vector2i depth_image_size, Matrix4f inverted_camera_pose,
                                      Vector4f inverted_camera_projection_parameters,
                                      float voxel_size_reciprocal, const Vector2f* ray_depth_range_image, float truncation_distance) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= depth_image_size.width || y >= depth_image_size.height) return;

	int locId = x + y * depth_image_size.width;
	int locId2 =
			(int) floor((float) x / ray_depth_image_subsampling_factor) + (int) floor((float) y / ray_depth_image_subsampling_factor) * depth_image_size.width;

	CastRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], block_visibility_types, x, y, voxels, index_data,
	                                              inverted_camera_pose, inverted_camera_projection_parameters, voxel_size_reciprocal, truncation_distance, ray_depth_range_image[locId2]);
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
__global__ void renderColour_device(Vector4u* outRendering, const Vector4f* ptsRay, const TVoxel* voxelData,
                                    const typename TIndex::IndexData* voxelIndex, Vector2i imgSize) {
	int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

	if (x >= imgSize.x || y >= imgSize.y) return;

	int locId = x + y * imgSize.x;

	Vector4f ptRay = ptsRay[locId];

	processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
}
}
