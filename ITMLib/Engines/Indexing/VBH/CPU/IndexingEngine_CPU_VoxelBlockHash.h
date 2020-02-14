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

#include "../IndexingEngine_VoxelBlockHash.h"


namespace ITMLib {

struct IndexingEngineFactory;
template<typename TVoxel>
class IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> :
		public IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CPU,
				IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>> {

	friend IndexingEngineFactory;
	friend IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CPU,
			IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>;

private:
	IndexingEngine() = default;
	template<typename TVoxelTarget, typename TVoxelSource, typename TMarkerFunctor>
	void AllocateUsingOtherVolume_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	                              VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume, TMarkerFunctor& marker_functor);

public:
	static IndexingEngine& Instance() {
		static IndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	IndexingEngine(IndexingEngine const&) = delete;
	void operator=(IndexingEngine const&) = delete;

	void AllocateHashEntriesUsingAllocationStateList(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;

	void
	AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;

	void AllocateBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                       const ORUtils::MemoryBlock<Vector3s>& new_block_positions,
	                       int new_block_count) override;

	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) override;
	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);


	bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code) override;

	void AllocateFromOtherVolume(VoxelVolume<TVoxel, VoxelBlockHash>* target_volume,
	                             VoxelVolume<TVoxel, VoxelBlockHash>* source_volume) override {
		AllocateUsingOtherVolume<TVoxel,TVoxel>(target_volume, source_volume);
	}

	void AllocateWarpVolumeFromOtherVolume(VoxelVolume<WarpVoxel, VoxelBlockHash>* target_volume,
	                             VoxelVolume<TVoxel, VoxelBlockHash>* source_volume) override {
		AllocateUsingOtherVolume<WarpVoxel,TVoxel>(target_volume, source_volume);
	}

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	                              VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume_Bounded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
	                                      const Extent3D& bounds);
	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume_OffsetAndBounded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	                                     VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
	                                     const Extent3D& source_bounds, const Vector3i& target_offset);

	void BuildUtilizedBlockListBasedOnVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                             const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(VoxelVolume<TVoxel, VoxelBlockHash>* volume);
};

extern template
class IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
extern template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;

} //namespace ITMLib
