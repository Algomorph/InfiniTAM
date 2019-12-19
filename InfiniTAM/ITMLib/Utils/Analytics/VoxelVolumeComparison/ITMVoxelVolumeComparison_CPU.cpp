//  ================================================================
//  Created by Gregory Kramida on 8/28/19.
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
#include "../../../ITMLibDefines.h"
#include "ITMVoxelVolumeComparison_CPU.tpp"

namespace ITMLib {

// region ======================= Instantiations with ITMVoxel =========================================================

template
bool contentAlmostEqual_CPU<ITMVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, ITMVoxelBlockHash, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, ITMVoxelBlockHash, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, ITMVoxelBlockHash, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool contentForFlagsAlmostEqual_CPU<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CPU_Verbose<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);


template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, ITMVoxelBlockHash, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* b,
		float tolerance);

//endregion
// region =================== Instantiations with ITMWarp ==============================================================

template
bool contentAlmostEqual_CPU<ITMWarp, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMWarp, ITMPlainVoxelArray, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, ITMVoxelBlockHash, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMWarp, ITMVoxelBlockHash, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, ITMVoxelBlockHash, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMWarp, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMWarp, ITMPlainVoxelArray, ITMVoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMWarp, ITMVoxelBlockHash, ITMPlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* b,
		float tolerance);

// endregion

} // namespace ITMLib 