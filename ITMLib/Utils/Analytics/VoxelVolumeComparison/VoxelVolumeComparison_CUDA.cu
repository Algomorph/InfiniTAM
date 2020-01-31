//  ================================================================
//  Created by Gregory Kramida on 9/25/19.
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
#include "VoxelVolumeComparison_CUDA.tcu"
#include "../../../ITMLibDefines.h"


namespace ITMLib {

// region ======================= Instantiations with ITMVoxel =========================================================

template
bool contentAlmostEqual_CUDA<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA_Verbose<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA<ITMVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, VoxelBlockHash>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA_Verbose<ITMVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, VoxelBlockHash>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA<ITMVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<ITMVoxel, VoxelBlockHash>* a, VoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentForFlagsAlmostEqual_CUDA<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CUDA_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool allocatedContentAlmostEqual_CUDA<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CUDA_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMVoxel, PlainVoxelArray>* a, VoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CUDA<ITMVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<ITMVoxel, VoxelBlockHash>* a, VoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

//endregion
// region =================== Instantiations with ITMWarp ==============================================================

template
bool contentAlmostEqual_CUDA<ITMWarp, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA_Verbose<ITMWarp, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA<ITMWarp, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA_Verbose<ITMWarp, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);


template
bool contentAlmostEqual_CUDA<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CUDA<ITMWarp, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* a, VoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool contentForFlagsAlmostEqual_CUDA<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CUDA_Verbose<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool allocatedContentAlmostEqual_CUDA<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CUDA_Verbose<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<ITMWarp, PlainVoxelArray>* a, VoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CUDA<ITMWarp, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* a, VoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);


// endregion

} // namespace ITMLib