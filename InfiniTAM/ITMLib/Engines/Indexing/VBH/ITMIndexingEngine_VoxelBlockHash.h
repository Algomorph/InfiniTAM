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
#include "../Interface/ITMIndexingEngine.h"
#include "../../Common/ITMWarpEnums.h"

namespace ITMLib{
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
class ITMIndexingEngine_VoxelBlockHash:
		public ITMIndexingEngineInterface<TVoxel> {

public:
	virtual void AllocateHashEntriesUsingLists(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
	                                           const HashEntryState* hashEntryStates_device,
	                                           Vector3s* blockCoordinates_device) = 0;

	virtual void AllocateHashEntriesUsingLists_SetVisibility(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
	                                                         const HashEntryState* hashEntryStates_device,
	                                                         Vector3s* blockCoordinates_device,
	                                                         uchar* hashBlockVisibilityTypes_device) = 0;
	virtual ITMHashEntry FindHashEntry(const ITMVoxelBlockHash& index, const Vector3s& coordinates) = 0;

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
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceTSDF,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* targetTSDF);
};

}// namespace ITMLib



