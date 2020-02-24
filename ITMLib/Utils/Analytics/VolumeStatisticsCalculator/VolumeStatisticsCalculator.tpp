//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include "VolumeStatisticsCalculator.h"
#include "../../../Objects/Volume/VoxelTypes.h"
#include "VolumeStatisticsCalculator_Functors.h"

using namespace ITMLib;


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeVoxelBounds(
		const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelBoundsFunctor<TVoxel, TIndex, TMemoryDeviceType>::Compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeAlteredVoxelBounds(
		VoxelVolume<TVoxel, TIndex>* volume) {
	ComputeConditionalVoxelBoundsFunctor<TVoxel, TMemoryDeviceType, IsAlteredStaticFunctor<TVoxel>> bounds_functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, bounds_functor);
	return bounds_functor.GetBounds();
}

//============================================== COUNT VOXELS ==========================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
int
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeAllocatedVoxelCount(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeAllocatedVoxelCountFunctor<TVoxel, TIndex, TMemoryDeviceType>::compute(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeVoxelWithFlagsCount(VoxelVolume<TVoxel,TIndex>* volume,
                                                                                          VoxelFlags flags) {
	return ComputeVoxelCountWithSpecificFlags<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>
	::compute(volume, flags);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeNonTruncatedVoxelCount(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelWithFlagsCount(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeNonTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(
			volume, VoxelFlags::VOXEL_NONTRUNCATED);

}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::
	compute(volume, VoxelFlags::VOXEL_TRUNCATED);
}

//========================================= HASH BLOCK STATISTICS ======================================================


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
int
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeAllocatedHashBlockCount(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::ComputeAllocatedHashBlockCount(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<int>
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashCodes(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
std::vector<Vector3s>
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedHashBlockPositions(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, TMemoryDeviceType>::GetAllocatedBlockPositions(volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::CountVoxelsWithSpecificSdfValue(
		VoxelVolume<TVoxel, TIndex>* volume,
		float value) {
	return ComputeVoxelCountWithSpecificValue<TVoxel::hasSDFInformation, TVoxel, TIndex, TMemoryDeviceType>::compute(
			volume, value);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
unsigned int
VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeAlteredVoxelCount(
		VoxelVolume<TVoxel, TIndex>* volume) {
	IsAlteredCountFunctor<TVoxel> functor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAll(volume, functor);
	return functor.GetCount();
}

// region ================================ VOXEL GRADIENT / WARP STATISTICS ============================================


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMin(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_UPDATE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMax(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_UPDATE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeWarpUpdateMean(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasWarpUpdate, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_UPDATE>::compute(
			volume);
}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMin(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MINIMUM, WARP_FRAMEWISE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMax(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MAXIMUM, WARP_FRAMEWISE>::compute(
			volume);
}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
double VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::ComputeFramewiseWarpMean(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, TMemoryDeviceType, MEAN, WARP_FRAMEWISE>::compute(
			volume);
}



// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
Vector6i VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::FindMinimumNonTruncatedBoundingBox(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return FlagMatchBBoxFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, TMemoryDeviceType>::
	compute(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}