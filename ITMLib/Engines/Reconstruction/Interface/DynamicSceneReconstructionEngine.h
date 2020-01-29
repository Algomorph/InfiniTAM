//  ================================================================
//  Created by Gregory Kramida on 5/7/18.
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
#pragma once

#include <cmath>

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../Common/ITMCommonFunctors.h"
#include "../../Common/ITMWarpEnums.h"

namespace ITMLib {
/** \brief
	Interface to engines implementing the main KinectFusion
	depth integration process in dynamic fusion settings that involve 2 different SDFs with different voxel types.

	These classes basically manage
	an ITMLib::Objects::ITMScene and fuse new image information
	into them.
*/
template<typename TVoxel, typename TWarp, typename TIndex>
class DynamicSceneReconstructionEngine {

public:

	DynamicSceneReconstructionEngine() = default;
	virtual ~DynamicSceneReconstructionEngine() = default;


	virtual void
	UpdateVisibleList(ITMVoxelVolume<TVoxel, TIndex>* scene, const ITMView* view, const ITMTrackingState* trackingState,
	                  const ITMRenderState* renderState, bool resetVisibleList) = 0;
	/**
	 * \brief Clears given scene, then uses the depth image from provided live view to generate an SDF
	 * voxel representation
	 * \param volume output scene
	 * \param view input view
	 * \param trackingState state of tracking
	 */
	virtual void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                        const ITMTrackingState* trackingState) = 0;

	/**
	 * \brief Clears given volume, then uses the depth image from provided live view to generate an SDF
	 * voxel representation
	 * \param[out] volume output volume
	 * \param[in] view input view
	 * \param[in] depth_camera_matrix current transformation matrix of the camera relative to world origin
	 */
	virtual void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) = 0;

	/**
	 * \brief Clears given scene, then uses the depth image from provided live view to generate a TSDF
	 * voxel representation. Allocation (if any is necessary by the data structure holding the TSDF) is done by
	 * expanding allocation required for just simple depth-image-based allocation by some kind of "sleeve", e.g.
	 * 1-ring of voxel blocks in the case of voxel hash blocks.
	 * \param[out] volume output volume
	 * \param temporaryAllocationVolume used for temporary allocation, may also be reset on being passed in
	 * \param[in] view input view
	 * \param[in] depth_camera_matrix current transformation matrix of the camera relative to world origin
	 */
	virtual void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, TIndex>* volume,
	                                                ITMVoxelVolume<TVoxel, TIndex>* temporaryAllocationVolume,
	                                                const ITMView* view,
	                                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) = 0;


	/**
	 * \brief Fuses the live scene into the canonical scene
	 * \details Operation happens after the motion is tracked, at this point sourceTsdfVolume should be as close to the canonical
	 * as possible
	 * \param targetTsdfVolume the canonical voxel grid, representing the state at the beginning of the sequence
	 * \param sourceTsdfVolume the live voxel grid, a TSDF generated from a single recent depth image
	 * \param liveSourceFieldIndex index of the sdf field to use at live scene voxels
	 */
	virtual void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, TIndex>* targetTsdfVolume,
	                                          ITMVoxelVolume<TVoxel, TIndex>* sourceTsdfVolume) = 0;

	/**
	 * \brief apply warp vectors to live scene: compute the the target SDF fields in live scene using trilinear lookup
	 * at corresponding composite warps from the source SDF fields in live scene
	 * \param canonicalScene canonical scene, where the warp update vectors are specified
	 * \param sourceTSDF live scene (voxel grid)
	 * \param sourceSdfIndex index of the source SDF field in each live voxel
	 * \param targetSdfIndex index of the target SDF field in each live voxel
	 */
	virtual void WarpScene_CumulativeWarps(ITMVoxelVolume<TWarp, TIndex>* warpField,
	                                       ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                       ITMVoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;

	/**
	 * \brief apply warp update vectors to live scene: compute the the target SDF fields in live scene using trilinear lookup
	 * at corresponding flow warps from the source SDF fields in live scene
	 * \param canonicalScene canonical scene, where the warp update vectors are specified
	 * \param liveScene live scene (voxel grid)
	 * \param sourceSdfIndex index of the source SDF field in each live voxel
	 * \param targetSdfIndex index of the target SDF field in each live voxel
	 */
	virtual void WarpScene_FramewiseWarps(ITMVoxelVolume<TWarp, TIndex>* warpField,
	                                 ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                 ITMVoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;

	//TODO: find a better way to write this doc (I think it's not clear from first glance)
	/**
	 * \brief apply warp update vectors to live scene: compute the the target SDF fields in live scene using trilinear
	 * lookup at corresponding position + warp updates from the source TSDF to the target TSDF grid
	 * \param warpField where the warp update / interpolation vectors are specified
	 * \param sourceTSDF this volume will be updated based on interpolated values
	 * \param targetTSDF this volume serves a source for interpolation
	 */
	virtual void WarpScene_WarpUpdates(ITMVoxelVolume<TWarp, TIndex>* warpField,
	                                   ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                   ITMVoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;
	/**
	 * \brief Update the voxel blocks by integrating depth and possibly color information from the given view. Assume
	 * camera is at world origin.
	 */
	virtual void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, TIndex>* volume, const ITMView* view) = 0;

	/** Update the voxel blocks by integrating depth and
	possibly colour information from the given view.*/
	virtual void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                               const ITMTrackingState* trackingState) = 0;

};
}//namespace ITMLib