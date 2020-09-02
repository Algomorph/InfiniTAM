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


//TODO: replace all functors that use atomics with reduction-type functors when PVA reduction is implemented

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeVoxelBounds(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelBoundsFunctor<TVoxel, TIndex, TMemoryDeviceType>::Compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeAlteredVoxelBounds(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	ComputeConditionalVoxelBoundsFunctor<TVoxel, TMemoryDeviceType, IsAlteredStaticFunctor<TVoxel>> bounds_functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilizedWithPosition(volume, bounds_functor);
	return bounds_functor.GetBounds();
}

//============================================== COUNT VOXELS ==========================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAllocatedVoxels(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelCountFunctor<TVoxel, TIndex, TMemoryDeviceType>::compute_allocated(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountUtilizedVoxels(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelCountFunctor<TVoxel, TIndex, TMemoryDeviceType>::compute_utilized(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithSpecifiedFlags(
		const VoxelVolume<TVoxel, TIndex>* volume,
		VoxelFlags flags) {
	return CountVoxelsWithSpecificFlagsFunctor<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>
	::compute(volume, flags);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountNonTruncatedVoxels(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return CountVoxelsWithSpecifiedFlags(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::SumNonTruncatedVoxelAbsSdf(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(
			volume, VoxelFlags::VOXEL_NONTRUNCATED);

}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::SumTruncatedVoxelAbsSdf(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::
	compute(volume, VoxelFlags::VOXEL_TRUNCATED);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithDepthWeightInRange(
		const VoxelVolume<TVoxel, TIndex>* volume, Extent2Di range) {
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
		const VoxelVolume<TVoxel, TIndex>* volume, Extent2Di range) {
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
		const VoxelVolume<TVoxel, TIndex>* volume,
		float value) {
	return CountVoxelsWithSpecificValueFunctor<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(volume, value);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAlteredVoxels(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	CountAlteredVoxelsFunctor<TVoxel, TMemoryDeviceType> functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, functor);
	return functor.GetCount();
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAlteredGradients(const VoxelVolume<TVoxel, TIndex>* volume) {
	CountAlteredGradientsFunctor<TVoxel, TMemoryDeviceType, TVoxel::hasWarpUpdate> functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, functor);
	return functor.GetCount();
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAlteredWarpUpdates(const VoxelVolume<TVoxel, TIndex>* volume) {
	CountAlteredWarpUpdatesFunctor<TVoxel, TMemoryDeviceType, TVoxel::hasWarpUpdate> functor;
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
			ReduceFunctorType, ReduceFunctorType, float>
			(position, volume, ignored_value);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMin(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_UPDATE>::compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMax(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_UPDATE>::compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMean(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_UPDATE>::compute(volume);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMin(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_FRAMEWISE>::compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMax(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_FRAMEWISE>::compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMean(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_FRAMEWISE>::compute(volume);
}

// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::FindMinimumNonTruncatedBoundingBox(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return FlagMatchBBoxFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}



// region ================================= HASH BLOCK STATISTICS ======================================================


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountAllocatedHashBlocks(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::ComputeAllocatedHashBlockCount(volume);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::CountUtilizedHashBlocks(const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::ComputeUtilizedHashBlockCount(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<int>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashBlockPositions(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedBlockPositions(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<int>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashCodes(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashCodes(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedHashBlockPositions(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetUtilizedBlockPositions(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyAnalysisFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(volume);
}




// endregion ===========================================================================================================
// region ================================ HISTOGRAMS ==================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Histogram
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateLengthHistogram_VolumeMax(const VoxelVolume<TVoxel, TIndex>* volume,
                                                                                               int bin_count, float& maximum) {
	return this->ComputeWarpUpdateLengthHistogram(volume, bin_count, maximum, false);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Histogram
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateLengthHistogram_ManualMax(const VoxelVolume<TVoxel, TIndex>* volume,
                                                                                               int bin_count, float maximum) {
	return this->ComputeWarpUpdateLengthHistogram(volume, bin_count, maximum, true);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Histogram
AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateLengthHistogram(
		const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float& maximum, bool use_manual_max) {

	auto& analytics_engine = AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance();
	unsigned int utilized_voxel_count = analytics_engine.CountUtilizedVoxels(volume);

	if (!use_manual_max) {
		Vector3i max_update_position;
		analytics_engine.ComputeWarpUpdateMaxAndPosition(maximum, max_update_position, volume);
	}

	Histogram histogram("Warp update length histogram", bin_count, utilized_voxel_count, TMemoryDeviceType);

	WarpHistogramFunctor<TVoxel, TMemoryDeviceType, TVoxel::hasWarpUpdate> warp_histogram_functor(histogram, maximum);
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, warp_histogram_functor);

	return histogram;
}

// endregion ===============================================================================================================