//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/26/20.
//  Copyright (c) 2020 Gregory Kramida
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
#include "Interface/RenderingEngineInterface.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "RenderingEngine_Specialized.h"

namespace ITMLib{

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
class RenderingEngine : public RenderingEngineBase<TVoxel, TIndex> {
private: // member variables
	internal::RenderingEngine_Specialized<TVoxel, TIndex, TMemoryDeviceType> specialized_engine;
public: // member functions
	void FindVisibleBlocks(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                       RenderState* render_state) const override;
	int CountVisibleBlocks(const VoxelVolume<TVoxel, TIndex>* volume, const RenderState* render_state, int min_block_list_id, int max_block_list_id) const override;
	void CreateExpectedDepths(const VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                          RenderState* render_state) const override;
	void RenderImage(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                 const RenderState* render_state, UChar4Image* output_image,
	                 IRenderingEngine::RenderImageType type, IRenderingEngine::RenderRaycastSelection raycast_type) const override;
	void FindSurface(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                 const RenderState* render_state) const override;
	void CreatePointCloud(VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state,
	                      RenderState* render_state, bool skipPoints) const override;
	void CreateICPMaps(VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state,
	                   RenderState* render_state) const override;
	void ForwardRender(const VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state,
	                   RenderState* render_state) const override;

};

} // namespace ITMLib