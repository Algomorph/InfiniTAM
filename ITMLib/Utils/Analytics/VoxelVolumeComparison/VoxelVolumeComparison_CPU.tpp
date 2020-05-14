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
#pragma once

#include "VoxelVolumeComparison_CPU.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../Engines/Traversal/Interface/TwoVolumeTraversal.h"
#include "VoxelVolumeComparison_Functors.h"


namespace ITMLib {
//region ================================= VOXEL VOLUME CONTENT COMPARISON FUNCTIONS ==================================



template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual_CPU(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                            ToleranceType tolerance) {
	VoxelEqualFunctor<TVoxel, ToleranceType> functor(tolerance);
	return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
	::template TraverseAndCompareAll(a, b, functor, false);
}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual_CPU_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                    ToleranceType tolerance, ExecutionMode execution_mode) {
	VoxelEqualVerboseFunctor<TVoxel, ToleranceType> functor(tolerance);
	switch (execution_mode) {
		case OPTIMIZED:
			return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
			::template TraverseAndCompareAllWithPosition<OPTIMIZED>(a, b, functor, true);
		case DIAGNOSTIC:
			return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
			::template TraverseAndCompareAllWithPosition<DIAGNOSTIC>(a, b, functor, true);
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized ExecutionMode.");
			return false;
	}

}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual_AsupersetB_CPU_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                    ToleranceType tolerance) {
	VoxelEqualVerboseFunctor<TVoxel, ToleranceType> functor(tolerance);
	return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
	::template TraverseAndCompareAllWithPosition_Volume1SupersetVolume2(a, b, functor, true);
}


// region ============================ COMPARISON OF VOXELS WITH SPECIFIC FLAGS WITHIN BOTH VOLUMES ====================

template<bool TBothVoxelTypesHaveFlags, typename TVoxel, typename TIndexA,
		typename TIndexB, typename ToleranceType, MemoryDeviceType TMemoryDeviceType>
struct FlaggedVoxelComparisonUtility;

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
struct FlaggedVoxelComparisonUtility<true, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU> {
	static
	bool compare(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	             ToleranceType tolerance) {
		VoxelEqualFunctor<TVoxel, ToleranceType> functor(tolerance);
		return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
		::template TraverseAndCompareMatchingFlags(a, b, flags, functor, false);
	}

	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED>
	static
	bool compare_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	                     ToleranceType tolerance) {
		VoxelEqualVerboseFunctor<TVoxel, ToleranceType> functor(tolerance);
		return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
		::template TraverseAndCompareMatchingFlagsWithPosition<TExecutionMode>(a, b, flags, functor, true);
	}

	static
	bool compare_AsupersetB_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	                     ToleranceType tolerance) {
		VoxelEqualVerboseFunctor<TVoxel, ToleranceType> functor(tolerance);
		return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
		::template TraverseAndCompareMatchingFlagsWithPosition_Volume1SupersetVolume2(a, b, flags, functor, true);
	}
};

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
struct FlaggedVoxelComparisonUtility<false, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU> {
	static
	bool compare(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	             ToleranceType tolerance) {
		std::cout << "Warning: flagged voxel comparison called on volumes where the voxel type doesn't have flags. "
		             "Defaulting to exact comparison for all voxels." << std::endl;
		return contentAlmostEqual_CPU(a, b, tolerance);
	}

	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED>
	static
	bool compare_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	                     ToleranceType tolerance) {
		std::cout << "Warning: flagged voxel comparison called on volumes where the voxel type doesn't have flags. "
		             " Defaulting to exact comparison for all voxels." << std::endl;
		return contentAlmostEqual_CPU_Verbose(a, b, tolerance);
	}

	bool compare_AsupersetB_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
	                                  ToleranceType tolerance){
		std::cout << "Warning: flagged voxel comparison called on volumes where the voxel type doesn't have flags. "
		             " Defaulting to exact comparison for voxels of volume b (assuming a should be a superset b)." << std::endl;
		return contentAlmostEqual_AsupersetB_CPU_Verbose(a, b, tolerance);
	}
};

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentForFlagsAlmostEqual_CPU(VoxelVolume<TVoxel,TIndexA>* a, VoxelVolume<TVoxel,TIndexB>* b, VoxelFlags flags, ToleranceType tolerance){
	return FlaggedVoxelComparisonUtility<TVoxel::hasSemanticInformation, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU>::
	compare(a, b, flags, tolerance);
}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentForFlagsAlmostEqual_CPU_Verbose(VoxelVolume<TVoxel,TIndexA>* a, VoxelVolume<TVoxel,TIndexB>* b, VoxelFlags flags, ToleranceType tolerance, ExecutionMode execution_mode){
	switch (execution_mode) {
		case OPTIMIZED:
			return FlaggedVoxelComparisonUtility<TVoxel::hasSemanticInformation, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU>::
			template compare_Verbose<OPTIMIZED>(a, b, flags, tolerance);
		case DIAGNOSTIC:
			return FlaggedVoxelComparisonUtility<TVoxel::hasSemanticInformation, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU>::
			template compare_Verbose<DIAGNOSTIC>(a, b, flags, tolerance);
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized ExecutionMode.");
			return false;
	}
}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentForFlagsAlmostEqual_AsupersetB_CPU_Verbose(VoxelVolume<TVoxel,TIndexA>* a, VoxelVolume<TVoxel,TIndexB>* b, VoxelFlags flags, ToleranceType tolerance){
	return FlaggedVoxelComparisonUtility<TVoxel::hasSemanticInformation, TVoxel, TIndexA, TIndexB, ToleranceType, MEMORYDEVICE_CPU>::
	compare_AsupersetB_Verbose(a, b, flags, tolerance);
}

// endregion ============================================================================================================

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool allocatedContentAlmostEqual_CPU(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                     ToleranceType tolerance){
	VoxelEqualFunctor<TVoxel, ToleranceType> functor(tolerance);
	return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
	::template TraverseAndCompareAllocated(a, b, functor);

}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool allocatedContentAlmostEqual_CPU_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                             ToleranceType tolerance){
	VoxelEqualVerboseFunctor<TVoxel, ToleranceType> functor(tolerance);
	return TwoVolumeTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, MEMORYDEVICE_CPU>
	::template TraverseAndCompareAllocatedWithPosition(a, b, functor);

}

//endregion
}