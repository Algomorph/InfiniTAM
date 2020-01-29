//  ================================================================
//  Created by Gregory Kramida on 5/28/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#include "DynamicSceneReconstructionEngine_CPU.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Functors.h"

using namespace ITMLib;

// region ========================================= PLAIN VOXEL ARRAY ==================================================


template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume_Helper(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view, Matrix4f camera_depth_matrix){

	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	Matrix4f camera_rgb_matrix;
	Vector4f projParams_d, projParams_rgb;

	if (TVoxel::hasColorInformation) camera_rgb_matrix = view->calib.trafo_rgb_to_depth.calib_inv * camera_depth_matrix;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = volume->sceneParams->narrow_band_half_width;
	int maxW = volume->sceneParams->max_integration_weight;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel* voxelArray = volume->localVBA.GetVoxelBlocks();

	const PlainVoxelArray::IndexData* arrayInfo = volume->index.GetIndexData();

	float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int locId = 0; locId < volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y *
	                            volume->index.GetVolumeSize().z; ++locId) {


		int z = locId / (volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y);
		int tmp = locId - z * volume->index.GetVolumeSize().x * volume->index.GetVolumeSize().y;
		int y = tmp / volume->index.GetVolumeSize().x;
		int x = tmp - y * volume->index.GetVolumeSize().x;

		Vector4f pt_model;


		pt_model.x = (float) (x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float) (y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float) (z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedLiveVoxelInfo<
				TVoxel::hasColorInformation,
				TVoxel::hasConfidenceInformation,
				TVoxel::hasSemanticInformation,
				TVoxel>::compute(voxelArray[locId], pt_model, camera_depth_matrix,
		                         projParams_d, camera_rgb_matrix, projParams_rgb, mu, maxW, depth, confidence,
		                         depthImgSize, rgb, rgbImgSize);
	}
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
		const ITMTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view){
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::FuseOneTsdfVolumeIntoAnother(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* liveScene) {
	TSDFFusionFunctor<TVoxel> fusionFunctor(canonicalScene->sceneParams->max_integration_weight);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CPU>::
	DualVoxelTraversal(liveScene, canonicalScene, fusionFunctor);
}

template<typename TVoxel, typename TWarp>
void
DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromView(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState) {
	GenerateTsdfVolumeFromView(scene, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromView(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	this->sceneManager.ResetScene(scene);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(scene, view, depth_camera_matrix);
}


template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromViewExpanded(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume, const ITMView* view,
		const Matrix4f& depth_camera_matrix) {
	GenerateTsdfVolumeFromView(volume, view, depth_camera_matrix);
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::UpdateVisibleList(
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	//do nothing
}


template<typename TVoxel, typename TWarp>
template<WarpType TWarpType>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::WarpScene(
		ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) {
//	 Clear out the flags at target index
	FieldClearFunctor<TVoxel> flagClearFunctor;
	ITMSceneTraversalEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::VoxelTraversal(targetTSDF,flagClearFunctor);

	TrilinearInterpolationFunctor<TVoxel, TWarp, PlainVoxelArray, TWarpType, MEMORYDEVICE_CPU>
			trilinearInterpolationFunctor(sourceTSDF, warpField);

//	 Interpolate to obtain the new live frame values (at target index)
	ITMDualSceneTraversalEngine<TVoxel, TWarp, PlainVoxelArray, PlainVoxelArray, MEMORYDEVICE_CPU>::
	DualVoxelPositionTraversal(targetTSDF, warpField, trilinearInterpolationFunctor);
}

template<typename TVoxel, typename TWarp>
void
DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::WarpScene_CumulativeWarps(
		ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::WarpScene_FramewiseWarps(
		ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_FLOW>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::WarpScene_WarpUpdates(
		ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}


// endregion ===========================================================================================================