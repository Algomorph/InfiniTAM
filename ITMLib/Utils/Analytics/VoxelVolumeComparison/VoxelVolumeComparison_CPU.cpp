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
#include "../../../GlobalTemplateDefines.h"
#include "../../../Engines/Traversal/CPU/TwoVolumeTraversal_CPU_PVA_to_VBH.h"
#include "../../../Engines/Traversal/CPU/TwoVolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../../Engines/Traversal/CPU/TwoVolumeTraversal_CPU_VoxelBlockHash.h"
#include "VoxelVolumeComparison_CPU.tpp"

namespace ITMLib {

// region ======================= Instantiations with TSDFVoxel =========================================================

template
bool contentAlmostEqual_CPU<TSDFVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<TSDFVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance, ExecutionMode execution_mode);

template
bool contentAlmostEqual_CPU<TSDFVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<TSDFVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance, ExecutionMode execution_mode);

template
bool contentAlmostEqual_CPU<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance, ExecutionMode execution_mode);

template
bool contentAlmostEqual_CPU<TSDFVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentForFlagsAlmostEqual_CPU<TSDFVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CPU_Verbose<TSDFVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance, ExecutionMode execution_mode);

template
bool contentForFlagsAlmostEqual_AsupersetB_CPU_Verbose<TSDFVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);


template
bool contentForFlagsAlmostEqual_CPU<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CPU_Verbose<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance, ExecutionMode execution_mode);


template
bool allocatedContentAlmostEqual_CPU<TSDFVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<TSDFVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance);


template
bool allocatedContentAlmostEqual_CPU<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<TSDFVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* a, VoxelVolume<TSDFVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<TSDFVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* a, VoxelVolume<TSDFVoxel, PlainVoxelArray>* b,
		float tolerance);

//endregion
// region =================== Instantiations with WarpVoxel ==============================================================

template
bool contentAlmostEqual_CPU<WarpVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* a, VoxelVolume<WarpVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<WarpVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* a, VoxelVolume<WarpVoxel, PlainVoxelArray>* b,
		float tolerance, ExecutionMode execution_mode);

template
bool contentAlmostEqual_CPU<WarpVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* a, VoxelVolume<WarpVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<WarpVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* a, VoxelVolume<WarpVoxel, VoxelBlockHash>* b,
		float tolerance, ExecutionMode execution_mode);

template
bool contentAlmostEqual_CPU<WarpVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* a, VoxelVolume<WarpVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<WarpVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* a, VoxelVolume<WarpVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<WarpVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* a, VoxelVolume<WarpVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<WarpVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* a, VoxelVolume<WarpVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<WarpVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* a, VoxelVolume<WarpVoxel, PlainVoxelArray>* b,
		float tolerance);

// endregion

} // namespace ITMLib 