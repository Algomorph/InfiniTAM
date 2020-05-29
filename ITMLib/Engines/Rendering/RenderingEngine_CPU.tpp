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

#include "RenderingEngine.h"
#include "../../Objects/Volume/VoxelBlockHash.h"
#include "Shared/RenderingEngine_Functors.h"

#include "../Traversal/Interface/HashTableTraversal.h"
#include "../Traversal/CPU/HashTableTraversal_CPU.h"

namespace ITMLib {
namespace internal {

template<class TVoxel>
void RenderingEngine_VoxelBlockHash_Specialized<TVoxel, MEMORYDEVICE_CPU>::FindVisibleBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, RenderState* render_state) const {
	FindVisibleBlocksFunctor<MEMORYDEVICE_CPU> functor(
			volume->index.GetVisibleBlockHashCodes(),
			volume->GetParameters().voxel_size,
			render_state->renderingRangeImage->dimensions,
			pose->GetM(),
			intrinsics->projectionParamsSimple.all
	);

	HashTableTraversalEngine<MEMORYDEVICE_CPU>::TraverseUtilizedWithHashCode(volume->index, functor);
	volume->index.SetVisibleBlockCount(functor.GetVisibleBlockCount());
}
} // namespace internal
} // namespace ITMLib