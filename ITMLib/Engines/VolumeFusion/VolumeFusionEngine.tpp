//  ================================================================
//  Created by Gregory Kramida on 1/30/20.
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

//local
#include "VolumeFusionEngine.h"
#include "VolumeFusionFunctors.h"
#include "../Indexing/Interface/IndexingEngine.h"
#include "../Traversal/Interface/TwoVolumeTraversal.h"

using namespace ITMLib;

//#define TRAVERSE_ALL_HASH_BLOCKS
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void VolumeFusionEngine<TVoxel, TIndex, TMemoryDeviceType>::FuseOneTsdfVolumeIntoAnother(
		VoxelVolume<TVoxel, TIndex>* target_volume, VoxelVolume<TVoxel, TIndex>* source_volume) {
	TSDFFusionFunctor<TVoxel, TMemoryDeviceType> fusion_functor(target_volume->parameters->max_integration_weight);
	TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndex, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAll(source_volume, target_volume, fusion_functor);
#else
	TraverseUtilized(source_volume, target_volume, fusion_functor);
#endif
}