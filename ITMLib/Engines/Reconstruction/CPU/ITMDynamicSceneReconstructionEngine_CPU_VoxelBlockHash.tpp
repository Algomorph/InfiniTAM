// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cfloat>
#include "ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Functors.h"

using namespace ITMLib;

// region ======================================= MODIFY VOXELS BASED ON VIEW ==========================================


template<typename TVoxel, typename TWarp>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume_Helper(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, Matrix4f depth_camera_matrix) {
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
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
		const ITMTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}


template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::IntegrateDepthImageIntoTsdfVolume(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}
// endregion ===========================================================================================================
// region =================================== FUSION ===================================================================

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::FuseOneTsdfVolumeIntoAnother(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* canonicalScene,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* liveScene) {
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateUsingOtherVolume(canonicalScene, liveScene);
	TSDFFusionFunctor<TVoxel> fusionFunctor(canonicalScene->sceneParams->max_integration_weight);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CPU>::
	DualVoxelTraversal(liveScene, canonicalScene, fusionFunctor);
}

template<typename TVoxel, typename TWarp>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromView(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
		const ITMTrackingState* trackingState) {
	GenerateTsdfVolumeFromView(scene, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromView(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	this->sceneManager.ResetScene(scene);
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateFromDepth(scene, view, depth_camera_matrix, false, false);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(scene, view, depth_camera_matrix);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::GenerateTsdfVolumeFromViewExpanded(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume, const ITMView* view,
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
// region ===================================== APPLY WARP/UPDATE TO LIVE ==============================================



/**
 * \brief Uses trilinear interpolation of the live frame at [canonical voxel positions + TWarpSource vector]
 *  (not complete warps) to generate a new live SDF grid in the target scene. Intended to be used at every iteration.
 * \details Assumes target frame is empty / has been reset.
 * Does 3 things:
 * <ol>
 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [current voxel position + warp vector],
 *       if there is a non-truncated voxel in the live frame, marks the block in target SDF volume at the current hash
 *       block position for allocation
 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
 * </ol>
 * \param canonicalScene canonical scene with warp vectors at each voxel.
 * \param liveScene source (current iteration) live scene
 * \param targetLiveScene target (next iteration) live scene
 * \tparam TWarp type of warp vector to use for interpolation
 */
template<typename TVoxel, typename TWarp>
template<WarpType TWarpType>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::WarpScene(
		ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {

	// Clear out the flags at target volume
	FieldClearFunctor<TVoxel> flagClearFunctor;
	ITMSceneTraversalEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::VoxelTraversal(targetTSDF, flagClearFunctor);

	// Allocate new blocks where necessary, filter based on flags from source
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().template AllocateFromWarpedVolume<TWarpType>(
			warpField, sourceTSDF, targetTSDF);

	TrilinearInterpolationFunctor<TVoxel, TWarp, VoxelBlockHash, TWarpType, MEMORYDEVICE_CPU>
			trilinearInterpolationFunctor(sourceTSDF, warpField);

	// Interpolate to obtain the new live frame values (at target index)
	ITMDualSceneTraversalEngine<TVoxel, TWarp, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CPU>::
	//DualVoxelPositionTraversal_DefaultForMissingSecondary(targetTSDF, warpField, trilinearInterpolationFunctor);
	DualVoxelPositionTraversal(targetTSDF, warpField, trilinearInterpolationFunctor);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::UpdateVisibleList(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
			.AllocateFromDepth(scene, view, trackingState, true, resetVisibleList);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::WarpScene_CumulativeWarps(
		ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::WarpScene_FramewiseWarps(
		ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_FLOW>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>::WarpScene_WarpUpdates(
		ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}





// endregion ===========================================================================================================
