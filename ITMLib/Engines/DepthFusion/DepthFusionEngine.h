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

#include "../../Objects/RenderStates/RenderState.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Objects/Tracking/CameraTrackingState.h"
#include "../../Objects/Views/ITMView.h"
#include "../Common/CommonFunctors.h"
#include "../Common/WarpType.h"

namespace ITMLib {
/** \brief
	Interface to engines implementing the main KinectFusion
	depth integration process in dynamic fusion settings that involve 2 different SDFs with different voxel types.

	These classes basically manage
	an ITMLib::Objects::ITMScene and fuse new image information
	into them.
*/
template<typename TVoxel, typename TWarp, typename TIndex>
class DepthFusionEngineInterface {

public:

	DepthFusionEngineInterface() = default;
	virtual ~DepthFusionEngineInterface() = default;


	virtual void
	UpdateVisibleList(VoxelVolume<TVoxel, TIndex>* scene, const ITMView* view, const CameraTrackingState* trackingState,
	                  const RenderState* renderState, bool resetVisibleList) = 0;
	/**
	 * \brief Clears given scene, then uses the depth image from provided live view to generate an SDF
	 * voxel representation
	 * \param volume output scene
	 * \param view input view
	 * \param trackingState state of tracking
	 */
	virtual void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                        const CameraTrackingState* trackingState) = 0;

	/**
	 * \brief Clears given volume, then uses the depth image from provided live view to generate an SDF
	 * voxel representation
	 * \param[out] volume output volume
	 * \param[in] view input view
	 * \param[in] depth_camera_matrix current transformation matrix of the camera relative to world origin
	 */
	virtual void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
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
	virtual void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, TIndex>* volume,
	                                                VoxelVolume<TVoxel, TIndex>* temporaryAllocationVolume,
	                                                const ITMView* view,
	                                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) = 0;

//	virtual void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view, const RenderState* renderState,
//			const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) = 0;

	/**
	 * \brief Update the voxel blocks by integrating depth and possibly color information from the given view. Assume
	 * camera is at world origin.
	 */
	virtual void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view) = 0;

	/** Update the voxel blocks by integrating depth and
	possibly colour information from the given view.*/
	virtual void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                               const CameraTrackingState* trackingState) = 0;

};


template<typename TVoxel, typename TWarp>
class DepthFusionEngine<TVoxel, TWarp, VoxelBlockHash>
		: public DepthFusionEngineInterface<TVoxel, TWarp, VoxelBlockHash> {
public:
	DepthFusionEngine() = default;
	~DepthFusionEngine() = default;

	void UpdateVisibleList(VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                       const CameraTrackingState* trackingState, const RenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const CameraTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                        VoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                       const CameraTrackingState* trackingState);
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                              Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

}//namespace ITMLib