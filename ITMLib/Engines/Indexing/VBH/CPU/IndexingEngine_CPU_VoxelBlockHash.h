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


namespace internal {

template<typename TVoxel>
struct SpecializedVoxelHashBlockManager<MEMORYDEVICE_CPU, TVoxel>{
	static HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hash_code);
	static bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hash_code);
	static void RebuildVisibleBlockList(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view, const Matrix4f& depth_camera_matrix);
};

template<typename TVoxelTarget, typename TVoxelSource>
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CPU, TVoxelTarget, TVoxelSource> {
	static inline
	void Execute(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
	             VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
	             const Extent3Di& source_bounds, const Vector3i& target_offset);
};

extern template
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CPU, TSDFVoxel, TSDFVoxel>;
extern template
struct AllocateUsingOtherVolume_OffsetAndBounded_Executor<MEMORYDEVICE_CPU, WarpVoxel, TSDFVoxel>;

} // namespace internal

} //namespace ITMLib
