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
#include "IndexingEngine_VoxelBlockHash_Internal.h"
#include "DIAGNOSTIC/IndexingEngine_VoxelBlockHash_DIAGNOSTIC.h"
#include "OPTIMIZED/IndexingEngine_VoxelBlockHash_OPTIMIZED.h"

namespace ITMLib {
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
class IndexingEngine<TVoxel, VoxelBlockHash, TMemoryDeviceType, TExecutionMode> :
		public IndexingEngineInterface<TVoxel, VoxelBlockHash>{
private: // member variables
	internal::IndexingEngine_VoxelBlockHash_ExecutionModeSpecialized<TMemoryDeviceType, TExecutionMode> execution_mode_specialized_engine;

protected: // member variables
	using IndexingEngineInterface<TVoxel,VoxelBlockHash>::parameters;
public: // member functions
	using IndexingEngineInterface<TVoxel,VoxelBlockHash>::GetParameters;
	using  IndexingEngineInterface<TVoxel,VoxelBlockHash>::IndexingEngineInterface;

	static IndexingEngine& Instance() {
		static IndexingEngine instance;
		return instance;
	}

	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates);
	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hash_code);
	bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code);
	void AllocateBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                       const ORUtils::MemoryBlock<Vector3s>& new_block_positions,
	                       int new_block_count = -1);
	void DeallocateBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                         const ORUtils::MemoryBlock<Vector3s>& coordinates_of_blocks_to_remove,
	                         int count_of_blocks_to_remove = -1);
	void ResetUtilizedBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;
	void ResetVisibleBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;
	void RebuildUtilizedBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
	void RebuildVisibleBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
	                             const Matrix4f& depth_camera_matrix = Matrix4f::Identity());
	void AllocateNearSurface(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
	                         const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void AllocateNearSurface(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
	                         const CameraTrackingState* tracking_state) override;
	void AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                       const View* view,
	                                       const CameraTrackingState* tracking_state) override;
	void AllocateHashEntriesUsingAllocationStateList(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
	void AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
	void AllocateGridAlignedBox(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const Extent3Di& box) override;

private: // member functions
	void ReallocateDeletedHashBlocks(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
	void ChangeVisibleBlockVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume, HashBlockVisibility visibility);
	template<typename TAllocationFunctor>
	void AllocateHashEntriesUsingAllocationStateList_Generic(VoxelVolume<TVoxel, VoxelBlockHash>* volume);

};


namespace  internal {

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource, typename TMarkerFunctor>
void AllocateUsingOtherVolume_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
                                      TMarkerFunctor& marker_functor);
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor;

} // namespace internal


} // namespace ITMLib



