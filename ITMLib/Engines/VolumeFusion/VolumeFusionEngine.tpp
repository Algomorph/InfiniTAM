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
#include "../Indexing/Interface/IndexingEngine.h"
#include "VolumeFusionFunctors.h"
#include "../Traversal/Interface/ITMSceneTraversal.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void VolumeFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::FuseOneTsdfVolumeIntoAnother(
		ITMVoxelVolume<TVoxel, TIndex>* targetVolume, ITMVoxelVolume<TVoxel, TIndex>* sourceVolume) {
	IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateUsingOtherVolume(targetVolume, sourceVolume);
	TSDFFusionFunctor<TVoxel, TMemoryDeviceType> fusionFunctor(targetVolume->sceneParams->max_integration_weight);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, TIndex, TIndex, TMemoryDeviceType>::
	DualVoxelTraversal(sourceVolume, targetVolume, fusionFunctor);
}