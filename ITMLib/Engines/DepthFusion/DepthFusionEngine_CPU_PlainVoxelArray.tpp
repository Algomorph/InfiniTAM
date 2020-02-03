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

#include "DepthFusionEngine_CPU.h"
#include "DepthFusionEngine_Shared.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"

using namespace ITMLib;

// region ========================================= PLAIN VOXEL ARRAY ==================================================


template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume_Helper(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view, Matrix4f camera_depth_matrix){

	const Vector2i rgbImgSize = view->rgb->noDims;
	const Vector2i depthImgSize = view->depth->noDims;
	const float voxelSize = volume->sceneParams->voxel_size;

	Matrix4f camera_rgb_matrix;
	if (TVoxel::hasColorInformation) camera_rgb_matrix = view->calib.trafo_rgb_to_depth.calib_inv * camera_depth_matrix;

	const Vector4f projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	const Vector4f projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	const float mu = volume->sceneParams->narrow_band_half_width;
	const int maxW = volume->sceneParams->max_integration_weight;

	const float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	const Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel* voxelArray = volume->localVBA.GetVoxelBlocks();

	const PlainVoxelArray::IndexData* arrayInfo = volume->index.GetIndexData();

	const float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(volume, voxelArray, arrayInfo, rgb, camera_rgb_matrix, depth, camera_depth_matrix, confidence)
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
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
		const CameraTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view){
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}

template<typename TVoxel, typename TWarp>
void
DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromView(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
		const CameraTrackingState* trackingState) {
	GenerateTsdfVolumeFromView(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromView(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix) {
	volume->Reset();
	this->IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, depth_camera_matrix);
}


template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::GenerateTsdfVolumeFromViewExpanded(
		VoxelVolume<TVoxel, PlainVoxelArray>* volume,
		VoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume, const ITMView* view,
		const Matrix4f& depth_camera_matrix) {
	GenerateTsdfVolumeFromView(volume, view, depth_camera_matrix);
}

template<typename TVoxel, typename TWarp>
void DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>::UpdateVisibleList(
		VoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view,
		const CameraTrackingState* trackingState,
		const RenderState* renderState, bool resetVisibleList) {
	//do nothing
}


// endregion ===========================================================================================================