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

#include "../../Interface/IndexingEngine.h"
#include "../IndexingEngine_VoxelBlockHash.h"
#include "../../../../Utils/WarpType.h"
#include "../../../../GlobalTemplateDefines.h"

namespace ITMLib {

struct IndexingEngineFactory;
template<typename TVoxel>
class IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> :
		public IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CUDA,
				IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> > {
	friend IndexingEngineFactory;
	friend IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CPU,
			IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>;

private:
	IndexingEngine() = default;


public:
	static IndexingEngine& Instance() {
		static IndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	IndexingEngine(IndexingEngine const&) = delete;
	void operator=(IndexingEngine const&) = delete;

	void AllocateHashEntriesUsingAllocationStateList_SetVisibility(VoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void BuildUtilizedBlockListBasedOnVisibility(VoxelVolume <TVoxel, VoxelBlockHash>* volume, const View* view,
	                                             const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(VoxelVolume <TVoxel, VoxelBlockHash>* volume);

	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) override;
	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);
	bool AllocateHashBlockAt(VoxelVolume <TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code) override;

};

extern template
class IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;

namespace internal {
template<typename TVoxelTarget, typename TVoxelSource>
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CUDA, TVoxelTarget, TVoxelSource> {
	static inline
	void Execute(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	             VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
	             const Extent3Di& source_bounds, const Vector3i& target_offset);
};

extern template
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CUDA, TSDFVoxel, TSDFVoxel>;
extern template
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CUDA, WarpVoxel, TSDFVoxel>;

} // namespace internal

} //namespace ITMLib

