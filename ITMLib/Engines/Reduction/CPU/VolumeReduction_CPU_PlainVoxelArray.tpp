//  ================================================================
//  Created by Gregory Kramida on 2/26/20.
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
//local
#include "VolumeReduction_CPU_PlainVoxelArray.h"
#include "../../../Utils/Math.h"

using namespace ITMLib;

template<typename TVoxel>
template<typename TRetrieveSingleFunctor, typename TReduceFunctor, typename TOutput>
TOutput VolumeReductionEngine<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::ReduceUtilized(
		Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
		ReductionResult<TOutput, PlainVoxelArray> ignored_value) {

	DIEWITHEXCEPTION_REPORTLOCATION("Not yet implemented.");
}