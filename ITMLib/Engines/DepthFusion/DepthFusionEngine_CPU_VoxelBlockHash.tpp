// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cfloat>
#include "DepthFusionEngine_CPU.h"
#include "DepthFusionEngine_Shared.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"

using namespace ITMLib;

// region ======================================= MODIFY VOXELS BASED ON VIEW ==========================================


template<typename TVoxel, typename TWarp>
void
DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume_Helper(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, Matrix4f depth_camera_matrix) {
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	Matrix4f RGB_camera_matrix;
	Vector4f projParams_d, projParams_rgb;

	if (TVoxel::hasColorInformation) RGB_camera_matrix = view->calib.trafo_rgb_to_depth.calib_inv * depth_camera_matrix;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = volume->sceneParams->narrow_band_half_width;
	int maxW = volume->sceneParams->max_integration_weight;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel* localVBA = volume->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = volume->index.GetEntries();

	int* visibleEntryHashCodes = volume->index.GetUtilizedBlockHashCodes();
	int visibleEntryCount = volume->index.GetUtilizedHashBlockCount();

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(visibleEntryCount, visibleEntryHashCodes, localVBA, hashTable, voxelSize,\
	rgb, rgbImgSize, depth, depthImgSize, mu, maxW, depth_camera_matrix, projParams_d, RGB_camera_matrix, projParams_rgb, confidence)
#endif
	for (int visibleHash = 0; visibleHash < visibleEntryCount; visibleHash++) {
		Vector3i globalPos;
		int hash = visibleEntryHashCodes[visibleHash];

		const ITMHashEntry& currentHashEntry = hashTable[hash];

		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= VOXEL_BLOCK_SIZE;

		TVoxel* localVoxelBlock = &(localVBA[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);

		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++)
			for (int y = 0; y < VOXEL_BLOCK_SIZE; y++)
				for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
					Vector4f pt_model;
					int locId;

					locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

					pt_model.x = (float) (globalPos.x + x) * voxelSize;
					pt_model.y = (float) (globalPos.y + y) * voxelSize;
					pt_model.z = (float) (globalPos.z + z) * voxelSize;
					pt_model.w = 1.0f;

					ComputeUpdatedLiveVoxelInfo<
							TVoxel::hasColorInformation,
							TVoxel::hasConfidenceInformation,
							TVoxel::hasSemanticInformation,
							TVoxel>::compute(localVoxelBlock[locId], pt_model, depth_camera_matrix,
					                         projParams_d, RGB_camera_matrix, projParams_rgb, mu, maxW, depth,
					                         confidence,
					                         depthImgSize, rgb, rgbImgSize);
				}
	}
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
		const ITMTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}


template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}
// endregion ===========================================================================================================
// region =================================== FUSION ===================================================================

template<typename TVoxel, typename TWarp>
void
DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromView(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
		const ITMTrackingState* trackingState) {
	GenerateTsdfVolumeFromView(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromView(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	volume->Reset();
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateFromDepth(volume, view, depth_camera_matrix, false, false);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, depth_camera_matrix);
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromViewExpanded(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		VoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume, const ITMView* view,
		const Matrix4f& depth_camera_matrix) {
	volume->Reset();
	temporaryAllocationVolume->Reset();
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer =
			IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	// Allocate blocks based on depth
	indexer.AllocateFromDepth(temporaryAllocationVolume, view, depth_camera_matrix, false, false);
	// Expand allocation by 1-ring of blocks
	indexer.AllocateUsingOtherVolumeExpanded(volume,temporaryAllocationVolume);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, depth_camera_matrix);
}

// endregion ===========================================================================================================



template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::UpdateVisibleList(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const RenderState* renderState, bool resetVisibleList) {
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateFromDepth(scene, view, trackingState, true, resetVisibleList);
}

