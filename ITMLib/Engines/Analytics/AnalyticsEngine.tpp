//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "AnalyticsEngine.h"
#include "../../Objects/Volume/VoxelTypes.h"
#include "AnalyticsEngine_Functors.h"
#include "../Reduction/Interface/VolumeReduction.h"

using namespace ITMLib;


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeVoxelBounds(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelBoundsFunctor<TVoxel, TIndex, TMemoryDeviceType>::Compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeAlteredVoxelBounds(
		VoxelVolume<TVoxel, TIndex>* volume) {
	ComputeConditionalVoxelBoundsFunctor<TVoxel, TMemoryDeviceType, IsAlteredStaticFunctor<TVoxel>> bounds_functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilizedWithPosition(volume, bounds_functor);
	return bounds_functor.GetBounds();
}

//============================================== COUNT VOXELS ==========================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAllocatedVoxels(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelCountFunctor<TVoxel, TIndex, TMemoryDeviceType>::compute_allocated(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountUtilizedVoxels(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelCountFunctor<TVoxel, TIndex, TMemoryDeviceType>::compute_utilized(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithSpecifiedFlags(
		VoxelVolume<TVoxel, TIndex>* volume,
		VoxelFlags flags) {
	return CountVoxelsWithSpecificFlagsFunctor<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>
	::compute(volume, flags);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountNonTruncatedVoxels(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return CountVoxelsWithSpecifiedFlags(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::SumNonTruncatedVoxelAbsSdf(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(
			volume, VoxelFlags::VOXEL_NONTRUNCATED);

}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::SumTruncatedVoxelAbsSdf(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::
	compute(volume, VoxelFlags::VOXEL_TRUNCATED);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithDepthWeightInRange(
		VoxelVolume<TVoxel, TIndex>* volume, Extent2Di range) {
	typedef RetrieveIsVoxelInDepthWeightRange<TVoxel, unsigned int, TVoxel::hasWeightInformation> RetrievalFunctorType;
	typedef ReduceSumFunctor<TVoxel, TIndex, unsigned int> ReduceFunctorType;
	RetrievalFunctorType functor{range};
	Vector3i position;
	return VolumeReductionEngine<TVoxel, TIndex, TMemoryDeviceType>::
	template ReduceUtilized<RetrievalFunctorType, ReduceFunctorType, ReduceFunctorType, unsigned int>
			(position, volume, functor);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountHashBlocksWithDepthWeightInRange(
		VoxelVolume<TVoxel, TIndex>* volume, Extent2Di range) {
	typedef RetrieveIsVoxelInDepthWeightRange<TVoxel, unsigned int, TVoxel::hasWeightInformation> RetrievalFunctorType;
	typedef ReduceBinAndFunctor<TVoxel, TIndex, unsigned int> BlockReduceFunctorType;
	typedef ReduceSumFunctor<TVoxel, TIndex, unsigned int> ResultReduceFunctorType;
	RetrievalFunctorType functor{range};
	Vector3i position;
	return VolumeReductionEngine<TVoxel, TIndex, TMemoryDeviceType>::
	template ReduceUtilized<RetrievalFunctorType, BlockReduceFunctorType, ResultReduceFunctorType, unsigned int>
			(position, volume, functor);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithSpecificSdfValue(
		VoxelVolume<TVoxel, TIndex>* volume,
		float value) {
	return CountVoxelsWithSpecificValueFunctor<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(
			volume, value);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAlteredVoxels(
		VoxelVolume<TVoxel, TIndex>* volume) {
	CountAlteredVoxelsFunctor<TVoxel> functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, functor);
	return functor.GetCount();
}

// region ================================ VOXEL GRADIENT / WARP STATISTICS ============================================
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMaxAndPosition(
		float& value, Vector3i& position, const VoxelVolume<TVoxel, TIndex>* volume) {
	ReductionResult<float, TIndex> ignored_value;
	typedef ReduceStatisticFunctor<TVoxel, TIndex, float, ITMLib::MAXIMUM> ReduceFunctorType;
	ignored_value.value = FLT_MIN;
	value = VolumeReductionEngine<TVoxel, TIndex, TMemoryDeviceType>::
	template ReduceUtilized<RetreiveWarpLengthFunctor<TVoxel, ITMLib::WARP_UPDATE>,
			ReduceFunctorType,ReduceFunctorType, float>
			(position, volume, ignored_value);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMin(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_UPDATE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMax(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_UPDATE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMean(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_UPDATE>::compute(
			volume);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMin(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_FRAMEWISE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMax(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_FRAMEWISE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMean(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_FRAMEWISE>::compute(
			volume);
}



// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::FindMinimumNonTruncatedBoundingBox(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return FlagMatchBBoxFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::
	compute(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}



// region ================================= HASH BLOCK STATISTICS ======================================================


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAllocatedHashBlocks(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::ComputeAllocatedHashBlockCount(volume);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountUtilizedHashBlocks(VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::ComputeUtilizedHashBlockCount(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<int>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashBlockPositions(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedBlockPositions(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<int>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashCodes(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashCodes(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashBlockPositions(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedBlockPositions(volume);
}



// endregion ===========================================================================================================