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
#include "../../../Utils/WarpType.h"

namespace ITMLib {


struct VoxelBlockMatch {
	int hash_code1;
	int hash_code2;
};


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
class IndexingEngine_VoxelBlockHash :
		public IndexingEngineInterface<TVoxel, VoxelBlockHash> {

public:

	virtual void AllocateHashEntriesUsingAllocationStateList(VoxelVolume <TVoxel, VoxelBlockHash>* volume) = 0;
	virtual void
	AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume <TVoxel, VoxelBlockHash>* volume) = 0;
	virtual HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) = 0;
	virtual bool AllocateHashBlockAt(VoxelVolume <TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) = 0;

	/**
	 * \brief Allocate all hash blocks at given coordinates
	 * \param volume - volume, where to allocate
	 * \param block_coordinates coordinates of blocks to allocate (in blocks, not voxels)
	 */
	virtual void
	AllocateBlockList(VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                  const ORUtils::MemoryBlock<Vector3s>& block_coordinates,
	                  int new_block_count) = 0;
	/**
	 * \brief Deallocate all hash blocks at given hash codes (hash codes here are treated as direct indices in the hash table, not buckets)
	 * \param volume - volume, where to deallocate
	 * \param block_coordinates coordinates of blocks to allocate (in blocks, not voxels)
	 */
	virtual void DeallocateBlockList(VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                 const ORUtils::MemoryBlock<int>& hash_codes_of_blocks_to_remove,
	                                 int count_blocks_to_remove) = 0;

	virtual void ResetUtilizedBlockList(VoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void AllocateNearSurface(
			VoxelVolume <TVoxel, VoxelBlockHash>* volume, const View* view,
			const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;

	void AllocateNearSurface(VoxelVolume <TVoxel, VoxelBlockHash>* volume, const View* view,
	                         const CameraTrackingState* tracking_state) override;

	void AllocateNearAndBetweenTwoSurfaces(VoxelVolume <TVoxel, VoxelBlockHash>* volume,
	                                       const View* view,
	                                       const CameraTrackingState* tracking_state) override;

	virtual void AllocateGridAlignedBox(VoxelVolume <TVoxel, VoxelBlockHash>* volume, const Extent3Di& box);


private:
	void ReallocateDeletedHashBlocks(VoxelVolume <TVoxel, VoxelBlockHash>* volume);

};

}// namespace ITMLib



