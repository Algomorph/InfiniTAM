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
#pragma once

//local
#include "../Interface/VolumeReduction.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../Shared/ReductionResult.h"
#include "../../../GlobalTemplateDefines.h"

//TODO: implement public static functions that currently throw the "Not yet implemented" exception

namespace ITMLib {
namespace internal {
template<typename TVoxel>
class VolumeReductionEngine_IndexSpecialized<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> {
public:
	template<typename TReduceStaticFunctor, typename TOutput, typename TRetrieveFunction>
	static TOutput ReduceUtilized_Generic(Vector3i& position, const VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                      TRetrieveFunction&& retrieve_function,
	                                      ReductionResult<TOutput, PlainVoxelArray> ignored_value) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not yet implemented.");
		return TOutput();
	}
};
} // namespace internal
} // namespace ITMLib