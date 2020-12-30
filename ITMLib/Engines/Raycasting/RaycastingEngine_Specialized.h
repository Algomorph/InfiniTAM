//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/28/20.
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

#include "Shared/RaycastingBlock.h"

namespace ITMLib {
namespace internal {

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
struct RaycastingEngine_Specialized;

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
struct RaycastingEngine_Specialized<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	inline void FindVisibleBlocks(VoxelVolume <TVoxel, PlainVoxelArray>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                              RenderState* render_state) const {}

	inline int CountVisibleBlocks(const VoxelVolume <TVoxel, PlainVoxelArray>* volume, int min_block_id, int max_block_id) const { return 1; }

	inline void CreateExpectedDepths(const VoxelVolume <TVoxel, PlainVoxelArray>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                                 RenderState* render_state) const;
};

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
struct RaycastingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	inline void FindVisibleBlocks(VoxelVolume <TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                              RenderState* render_state) const;

	inline int CountVisibleBlocks(const VoxelVolume <TVoxel, VoxelBlockHash>* volume, int min_block_id, int max_block_id) const;

	inline void CreateExpectedDepths(const VoxelVolume <TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
	                                 RenderState* render_state) const;
};

} // namespace internal
} // namespace ITMLib
