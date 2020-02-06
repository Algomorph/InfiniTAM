//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../Interface/IndexingEngine.h"
#include "../../Common/WarpType.h"

namespace ITMLib {
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
class IndexingEngine_VoxelBlockHash :
		public IndexingEngineInterface<TVoxel, VoxelBlockHash> {

public:
	virtual void AllocateHashEntriesUsingLists(VoxelVolume<TVoxel, VoxelBlockHash>* volume) = 0;
	virtual void AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) = 0;
	virtual HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) = 0;
	virtual bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) = 0;

/**
 * \brief method which looks at voxel grid with warps and an SDF voxel grid and allocates all hash blocks in the
 * SDF grid where warp vectors are pointing to (if not already allocated).
 * \details scans each (allocated) voxel in the SDF voxel grid, checks the warp vector at the corresponding location,
 * finds the voxel where the warp vector is pointing to, and, if the hash block for that voxel is not yet allocated,
 * allocates it.
 * \param warpField voxel grid where each voxel has a .warp Vector3f field defined
 * \param sourceTsdf sdf grid whose hash blocks to allocate if needed
 * \param sourceSdfIndex index of the sdf / flag field to use in the sdfScene
 * \tparam TVoxelBType the type of warp vector to use
 */
	template<WarpType TWarpType, typename TWarp>
	void AllocateFromWarpedVolume(
			VoxelVolume<TWarp, VoxelBlockHash>* warpField,
			VoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
			VoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF);

	void AllocateFromDepth(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                       bool only_update_visible_list = false, bool resetVisibleList = false) override;

	void AllocateFromDepth(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const CameraTrackingState* tracking_state, bool onlyUpdateVisibleList,
	                       bool resetVisibleList) override;

	void AllocateFromDepthAndSdfSpan(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                 const CameraTrackingState* tracking_state,
	                                 const ITMView* view) override;

private:
	void ReallocateDeletedHashBlocks(VoxelVolume<TVoxel, VoxelBlockHash>* volume);

};

}// namespace ITMLib



