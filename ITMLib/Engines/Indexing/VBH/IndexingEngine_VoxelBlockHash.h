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

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TDerivedClass>
class IndexingEngine_VoxelBlockHash :
		public IndexingEngineInterface<TVoxel, VoxelBlockHash> {

public: // member functions

	void AllocateHashEntriesUsingAllocationStateList(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
	virtual void
	AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) = 0;
	virtual HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) = 0;
	virtual bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) = 0;

	/**
	 * \brief Allocate all hash blocks at the first [0, new_block_count) of the given block coordinates
	 * \param volume volume where to allocate
	 * \param new_block_positions list of coordinates of blocks to allocate (coordinates to be specified in blocks, not voxels or meters)
	 * \param new_block_count only the first [0, new_block_count) new_block_positions will be used.
	 * If -1 is passed, new_block_positions.size() will be used.
	 */
	void AllocateBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                       const ORUtils::MemoryBlock<Vector3s>& new_block_positions,
	                       int new_block_count = -1);
	/**
	 * \brief Deallocate all hash blocks at the first [0, count_of_blocks_to_remove) of the given block coordinates
	 * \param volume volume where to deallocate
	 * \param coordinates_of_blocks_to_remove coordinates (specified in blocks, not voxels or meters) from which to remove blocks
	 * \param count_of_blocks_to_remove only the first [0, count_of_blocks_to_remove) coordinates_of_blocks_to_remove will be used.
	 * If -1 is passed, coordinates_of_blocks_to_remove.size() will be used.
	 */
	void DeallocateBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                 const ORUtils::MemoryBlock<Vector3s>& coordinates_of_blocks_to_remove,
	                                 int count_of_blocks_to_remove = -1);

	virtual void ResetUtilizedBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;

	void RebuildUtilizedBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume);

	void AllocateNearSurface(
			VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
			const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;

	void AllocateNearSurface(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
	                         const CameraTrackingState* tracking_state) override;

	void AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                       const View* view,
	                                       const CameraTrackingState* tracking_state) override;

	virtual void AllocateGridAlignedBox(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const Extent3Di& box);


private:
	void ReallocateDeletedHashBlocks(VoxelVolume<TVoxel, VoxelBlockHash>* volume);

};

namespace internal {
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource, typename TMarkerFunctor>
void AllocateUsingOtherVolume_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
                                      TMarkerFunctor& marker_functor);
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor;
} // namespace internal


} // namespace ITMLib



