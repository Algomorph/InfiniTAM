//  ================================================================
//  Created by Gregory Kramida on 10/30/19.
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

//stdlib
#include <limits>

//local
#ifdef __CUDACC__
#include "../../Utils/CUDAUtils.h"
#include "../Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h"
#include "../Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#endif

#include "../Common/WarpAccessFunctors.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../Reduction/Interface/VolumeReduction.h"
#include "../Reduction/Interface/BlockVolumeReduction.h"
#include "../Traversal/Interface/HashTableTraversal.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../../Utils/Enums/WarpType.h"
#include "../../Utils/Analytics/Statistics.h"
#include "../../Utils/Collections/MemoryBlock_StdContainer_Convertions.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../Utils/Collections/OperationsOnSTLContainers.h"

using namespace ITMLib;

// region =========================================== ATOMIC VECTOR/GRADIENT FIELD MIN/MEAN/MAX ========================
// TODO: replace with reduction functors when PVA reduction is implemented
template<typename TVoxel, Statistic TStatistic, WarpType TWarpType>
struct HandleVectorLengthAggregate;
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
		ATOMIC_ADD(count, 1u);
	}
};

template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));

	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));

	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	AggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, const TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};

template<bool hasWarp, typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor;

template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor<false, TVoxel, TIndex, TDeviceType, TStatistic, TWarpType> {
	static int compute(const VoxelVolume<TVoxel, TIndex>* volume) {
		DIEWITHEXCEPTION("Voxels need to have warp information to get warp statistics.");
		return 0;
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor<true, TVoxel, TIndex, TDeviceType, TStatistic, TWarpType> {

	~ComputeWarpLengthStatisticFunctor() {
		CLEAN_UP_ATOMIC(aggregate);CLEAN_UP_ATOMIC(count);
	}

	static double compute(const VoxelVolume<TVoxel, TIndex>* volume) {
		ComputeWarpLengthStatisticFunctor instance;
		INITIALIZE_ATOMIC(double, instance.aggregate, 0.0);
		INITIALIZE_ATOMIC(unsigned int, instance.count, 0u);
		VolumeTraversalEngine<TVoxel, TIndex, TDeviceType>::TraverseUtilized(volume, instance);
		double aggregate = GET_ATOMIC_VALUE_CPU(instance.aggregate);
		unsigned int count = GET_ATOMIC_VALUE_CPU(instance.count);
		if (TStatistic == MEAN) {
			return aggregate / count;
		} else {
			return aggregate;
		}
	}

	DECLARE_ATOMIC(double, aggregate);
	DECLARE_ATOMIC(unsigned int, count);
	Vector3i position;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		HandleVectorLengthAggregate<TVoxel, TStatistic, TWarpType>::AggregateStatistic(aggregate, count, voxel);
	}
};

//endregion
// region =========================================== REDUCTION VECTOR/GRADIENT FIELD MIN/MEAN/MAX =====================

template<typename TVoxel, ITMLib::WarpType TWarpType>
struct RetreiveWarpLengthFunctor {
	_CPU_AND_GPU_CODE_
	inline static float retrieve(const TVoxel& voxel) {
		return ORUtils::length(WarpAccessStaticFunctor<TVoxel, TWarpType>::GetWarp(voxel));
	}
};

template<typename TVoxel, typename TIndex, typename TOutput, ITMLib::Statistic TStatistic>
struct ReduceStatisticFunctor;

template<typename TVoxel, typename TIndex, typename TOutput>
struct ReduceStatisticFunctor<TVoxel, TIndex, TOutput, ITMLib::MAXIMUM> {
public:
	_CPU_AND_GPU_CODE_
	inline static const ReductionResult<TOutput, TIndex>& reduce(
			const ReductionResult<TOutput, TIndex>& item1, const ReductionResult<TOutput, TIndex>& item2) {
		return (item1.value > item2.value) ? item1 : item2;
	}
};

// endregion

//region ================== **** VOXEL COUNTS **** =====================================================================

//region ============================ REDUCTION COUNT FUNCTORS =========================================================

template<typename TVoxel, typename TOutput, bool ThasDepthInformation>
struct RetrieveIsVoxelInDepthWeightRange;

template<typename TVoxel, typename TOutput>
struct RetrieveIsVoxelInDepthWeightRange<TVoxel, TOutput, true> {
	Extent2Di range;
	_CPU_AND_GPU_CODE_
	inline TOutput retrieve(const TVoxel& voxel) const {
		return (range.from <= voxel.w_depth) && (voxel.w_depth < range.to);
	}
};

template<typename TVoxel, typename TOutput>
struct RetrieveIsVoxelInDepthWeightRange<TVoxel, TOutput, false> {
	Extent2Di range;
	_CPU_AND_GPU_CODE_
	inline TOutput retrieve(const TVoxel& voxel) const {
		DIEWITHEXCEPTION_REPORTLOCATION("Voxel doesn't have depth information.");
		return TOutput();
	}
};

template<typename TVoxel, typename TOutput, bool ThasDepthInformation>
struct RetrieveIsVoxelNotInDepthWeightRange;

template<typename TVoxel, typename TOutput>
struct RetrieveIsVoxelNotInDepthWeightRange<TVoxel, TOutput, true> {
	Extent2Di range;
	_CPU_AND_GPU_CODE_
	inline TOutput retrieve(const TVoxel& voxel) const {
		return (range.from > voxel.w_depth) || (voxel.w_depth >= range.to);
	}
};

template<typename TVoxel, typename TOutput>
struct RetrieveIsVoxelNotInDepthWeightRange<TVoxel, TOutput, false> {
	Extent2Di range;
	_CPU_AND_GPU_CODE_
	inline TOutput retrieve(const TVoxel& voxel) const {
		DIEWITHEXCEPTION_REPORTLOCATION("Voxel doesn't have depth information.");
		return TOutput();
	}
};

template<typename TVoxel, typename TIndex, typename TOutput>
struct ReduceSumFunctor {
public:
	_CPU_AND_GPU_CODE_
	inline static ReductionResult<TOutput, TIndex> reduce(
			const ReductionResult<TOutput, TIndex>& item1, const ReductionResult<TOutput, TIndex>& item2) {
		return {item1.value + item2.value, 0u, 0};
	}
};

template<typename TVoxel, typename TIndex, typename TOutput>
struct ReduceBinOrFunctor {
public:
	_CPU_AND_GPU_CODE_
	inline static ReductionResult<TOutput, TIndex> reduce(
			const ReductionResult<TOutput, TIndex>& item1, const ReductionResult<TOutput, TIndex>& item2) {
		return {item1.value | item2.value, 0u, 0};
	}
};

template<typename TVoxel, typename TIndex, typename TOutput>
struct ReduceBinAndFunctor {
public:
	_CPU_AND_GPU_CODE_
	inline static ReductionResult<TOutput, TIndex> reduce(
			const ReductionResult<TOutput, TIndex>& item1, const ReductionResult<TOutput, TIndex>& item2) {
		return {item1.value & item2.value, 0u, 0};
	}
};


//endregion ============================================================================================================
//region =========================== COUNT ALL VOXELS ==================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	inline
	static unsigned int compute_allocated(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		const PlainVoxelArray& index = volume->index;
		return static_cast<unsigned int>(index.GetVolumeSize().x * index.GetVolumeSize().y * index.GetVolumeSize().z);
	}

	inline
	static unsigned int compute_utilized(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return compute_allocated(volume);
	}
};
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyAnalysisFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	inline
	static unsigned int compute_allocated(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		return static_cast<unsigned int>(HashOnlyAnalysisFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType>
		::ComputeAllocatedHashBlockCount(volume)) * VOXEL_BLOCK_SIZE3;
	}

	inline
	static unsigned int compute_utilized(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		return volume->index.GetUtilizedBlockCount() * VOXEL_BLOCK_SIZE3;
	}
};
//endregion
//region ========================= COUNT VOXELS WITH SPECIFIC SDF VALUE ================================================

template<bool hasSDFInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificValueFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificValueFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit CountVoxelsWithSpecificValueFunctor(float value) : value(value) {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	}

	~CountVoxelsWithSpecificValueFunctor() {
		CLEAN_UP_ATOMIC(count);
	}

	static int compute(const VoxelVolume<TVoxel, TIndex>* volume, float value) {
		CountVoxelsWithSpecificValueFunctor instance(value);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, instance);
		return GET_ATOMIC_VALUE_CPU(instance.count);
	}

	DECLARE_ATOMIC(unsigned int, count);
	float value = 0.0f;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (TVoxel::valueToFloat(voxel.sdf) == value) {
			ATOMIC_ADD(count, 1u);
		}
	}
};

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificValueFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {
	static int compute(const VoxelVolume<TVoxel, TIndex>* volume, float value) {
		DIEWITHEXCEPTION("Voxel volume issued to count voxels with specific SDF value appears to have no sdf information. "
		                 "Voxels in volume need to have sdf information for this operation to work.");
		return 0;
	}
};
//endregion
//region ================================ COUNT VOXELS WITH SPECIFIC FLAGS =============================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificFlagsFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificFlagsFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit CountVoxelsWithSpecificFlagsFunctor(VoxelFlags flags) : flags(flags) {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	}

	~CountVoxelsWithSpecificFlagsFunctor() {
		CLEAN_UP_ATOMIC(count);
	}

	static int compute(const VoxelVolume<TVoxel, TIndex>* volume, VoxelFlags flags) {
		CountVoxelsWithSpecificFlagsFunctor instance(flags);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, instance);
		return GET_ATOMIC_VALUE_CPU(instance.count);
	}

	DECLARE_ATOMIC(unsigned int, count);
	VoxelFlags flags;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (voxel.flags == flags) {
			ATOMIC_ADD(count, 1u);
		}
	}
};

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct CountVoxelsWithSpecificFlagsFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {
	static int compute(const VoxelVolume<TVoxel, TIndex>* volume, float value) {
		DIEWITHEXCEPTION(
				"Voxel volume issued to count voxels with specific semantic flags appears to have no semantic information. "
				"Voxels in volume need to have semantic flags field for this operation to work.");
		return 0;
	}
};
//endregion
//region ================================== COUNT ALTERED VOXELS =======================================================

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct CountAlteredVoxelsFunctor {
	CountAlteredVoxelsFunctor() {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	};

	~CountAlteredVoxelsFunctor() {
		CLEAN_UP_ATOMIC(count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (isAltered(voxel)) {
			ATOMIC_ADD(count, 1u);
		}
	}

	unsigned int GetCount() {
		return GET_ATOMIC_VALUE_CPU(count);
	}

	DECLARE_ATOMIC(unsigned int, count);
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, bool THasWarpUpdate>
struct CountAlteredGradientsFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct CountAlteredGradientsFunctor<TVoxel, TMemoryDeviceType, true> {
	CountAlteredGradientsFunctor() {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	};

	~CountAlteredGradientsFunctor() {
		CLEAN_UP_ATOMIC(count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (voxel.gradient0 != Vector3f(0.0f)) {
			ATOMIC_ADD(count, 1u);
		}
	}

	unsigned int GetCount() {
		return GET_ATOMIC_VALUE_CPU(count);
	}

	DECLARE_ATOMIC(unsigned int, count);
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct CountAlteredGradientsFunctor<TVoxel, TMemoryDeviceType, false> {
	CountAlteredGradientsFunctor() {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Trying to count altered gradients on voxel type that has no warp updates (and, hence, no gradients). Aborting.");
	};

	~CountAlteredGradientsFunctor() {

	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
	}

	unsigned int GetCount() {
		return 0;
	}
};


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, bool THasWarpUpdate>
struct CountAlteredWarpUpdatesFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct CountAlteredWarpUpdatesFunctor<TVoxel, TMemoryDeviceType, true> {
	CountAlteredWarpUpdatesFunctor() {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	};

	~CountAlteredWarpUpdatesFunctor() {
		CLEAN_UP_ATOMIC(count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (voxel.warp_update != Vector3f(0.0f)) {
			ATOMIC_ADD(count, 1u);
		}
	}

	unsigned int GetCount() {
		return GET_ATOMIC_VALUE_CPU(count);
	}

	DECLARE_ATOMIC(unsigned int, count);
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct CountAlteredWarpUpdatesFunctor<TVoxel, TMemoryDeviceType, false> {
	CountAlteredWarpUpdatesFunctor() {
		DIEWITHEXCEPTION_REPORTLOCATION("Trying to count altered warp updates on voxel type that has no warp updates. Aborting.");
	};

	~CountAlteredWarpUpdatesFunctor() {

	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
	}

	unsigned int GetCount() {
		return 0;
	}
};


//endregion

//endregion

//region =========================================== SUM OF TOTAL SDF ==================================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {

	static double compute(const VoxelVolume<TVoxel, TIndex>* volume, ITMLib::VoxelFlags voxel_flags) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Voxels need to have semantic information to be marked as truncated or non-truncated.");
		return 0.0;
	}
};
template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit SumSDFFunctor(ITMLib::VoxelFlags voxel_flags) : voxel_flags(voxel_flags) {
		INITIALIZE_ATOMIC(double, sum, 0.0);
	}

	~SumSDFFunctor() {
		CLEAN_UP_ATOMIC(sum);
	}

	static double compute(const VoxelVolume<TVoxel, TIndex>* volume, ITMLib::VoxelFlags voxel_flags) {
		SumSDFFunctor instance(voxel_flags);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilized(volume, instance);
		return GET_ATOMIC_VALUE_CPU(instance.sum);
	}

	DECLARE_ATOMIC(double, sum);
	ITMLib::VoxelFlags voxel_flags;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel) {
		if (voxel_flags == (ITMLib::VoxelFlags) voxel.flags) {
			ATOMIC_ADD(sum, std::abs(static_cast<double>(TVoxel::valueToFloat(voxel.sdf))));
		}
	}
};
//endregion

//region ======================================= BOUNDING BOX COMPUTATIONS =============================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {
	static Extent3Di compute(const VoxelVolume<TVoxel, TIndex>* volume, ITMLib::VoxelFlags voxel_flags) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Voxels need to have semantic information to be marked as truncated or non-truncated.");
		return {};
	}
};
template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit FlagMatchBBoxFunctor(ITMLib::VoxelFlags voxel_flags) : voxel_flags(voxel_flags) {
		INITIALIZE_ATOMIC(int, min_x, INT_MAX);
		INITIALIZE_ATOMIC(int, min_y, INT_MAX);
		INITIALIZE_ATOMIC(int, min_z, INT_MAX);
		INITIALIZE_ATOMIC(int, max_x, INT_MIN);
		INITIALIZE_ATOMIC(int, max_y, INT_MIN);
		INITIALIZE_ATOMIC(int, max_z, INT_MIN);
	}

	~FlagMatchBBoxFunctor() {
		//@formatter:off
		CLEAN_UP_ATOMIC(min_x);
		CLEAN_UP_ATOMIC(min_y);
		CLEAN_UP_ATOMIC(min_z);
		CLEAN_UP_ATOMIC(max_x);
		CLEAN_UP_ATOMIC(max_y);
		CLEAN_UP_ATOMIC(max_z);
		//@formatter:on
	}


	static Extent3Di compute(const VoxelVolume<TVoxel, TIndex>* volume, ITMLib::VoxelFlags voxel_flags) {
		FlagMatchBBoxFunctor instance(voxel_flags);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseUtilizedWithPosition(volume, instance);
		return {GET_ATOMIC_VALUE_CPU(instance.min_x),
		        GET_ATOMIC_VALUE_CPU(instance.min_y),
		        GET_ATOMIC_VALUE_CPU(instance.min_z),
		        GET_ATOMIC_VALUE_CPU(instance.max_x),
		        GET_ATOMIC_VALUE_CPU(instance.max_y),
		        GET_ATOMIC_VALUE_CPU(instance.max_z)};
	}

	DECLARE_ATOMIC(int, min_x);
	DECLARE_ATOMIC(int, min_y);
	DECLARE_ATOMIC(int, min_z);
	DECLARE_ATOMIC(int, max_x);
	DECLARE_ATOMIC(int, max_y);
	DECLARE_ATOMIC(int, max_z);
	ITMLib::VoxelFlags voxel_flags;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TVoxel& voxel, Vector3i voxel_position) {
		if (voxel_flags == (ITMLib::VoxelFlags) voxel.flags) {
			ATOMIC_MIN(min_x, voxel_position.x);
			ATOMIC_MIN(min_y, voxel_position.y);
			ATOMIC_MIN(min_z, voxel_position.z);
			ATOMIC_MAX(max_x, voxel_position.x);
			ATOMIC_MAX(max_y, voxel_position.y);
			ATOMIC_MAX(max_z, voxel_position.z);
		}
	}
};
//endregion
//region ===================================== HASH BLOCK STATS ========================================================

template<MemoryDeviceType TMemoryDeviceType>
struct AllocatedHashBlockCountFunctor {
	AllocatedHashBlockCountFunctor() {
		INITIALIZE_ATOMIC(unsigned int, allocated_hash_block_count, 0);
	}

	~AllocatedHashBlockCountFunctor() {
		CLEAN_UP_ATOMIC(allocated_hash_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& entry, const int hash_code) {
		if (entry.ptr >= 0) {
			ATOMIC_ADD(allocated_hash_block_count, 1u);
		}
	}

	unsigned int get_count() {
		return GET_ATOMIC_VALUE_CPU(allocated_hash_block_count);
	}

private:
	DECLARE_ATOMIC(unsigned int, allocated_hash_block_count);
};

template<MemoryDeviceType TMemoryDeviceType>
struct AllocatedHashesAggregationFunctor {
	AllocatedHashesAggregationFunctor(unsigned int allocated_block_count) : hash_codes(allocated_block_count,
	                                                                                   TMemoryDeviceType) {
		INITIALIZE_ATOMIC(unsigned int, current_fill_index, 0);
		hash_codes_device = hash_codes.GetData(TMemoryDeviceType);
	}


	~AllocatedHashesAggregationFunctor() {
		CLEAN_UP_ATOMIC(current_fill_index);
	}


	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& entry, const int hash_code) {
		if (entry.ptr >= 0) {
			unsigned int index = ATOMIC_ADD(current_fill_index, 1u);
			hash_codes_device[index] = hash_code;
		}
	}

	std::vector<int> data() {
		return ORUtils_MemoryBlock_to_std_vector(hash_codes, TMemoryDeviceType);
	}

private:
	int* hash_codes_device;
	ORUtils::MemoryBlock<int> hash_codes;
	DECLARE_ATOMIC(unsigned int, current_fill_index);
};


template<MemoryDeviceType TMemoryDeviceType>
struct BlockPositionAggregationFunctor {
	BlockPositionAggregationFunctor(unsigned int block_count) : block_positions(block_count,
	                                                                            TMemoryDeviceType) {
		INITIALIZE_ATOMIC(unsigned int, current_fill_index, 0);
		block_positions_device = block_positions.GetData(TMemoryDeviceType);
	}


	~BlockPositionAggregationFunctor() {
		CLEAN_UP_ATOMIC(current_fill_index);
	}


	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& entry, const int hash_code) {
		if (entry.ptr >= 0) {
			unsigned int index = ATOMIC_ADD(current_fill_index, 1u);
			block_positions_device[index] = entry.pos;
		}
	}

	std::vector<Vector3s> data() {
		return ORUtils_MemoryBlock_to_std_vector(block_positions, TMemoryDeviceType);
	}

private:
	Vector3s* block_positions_device;
	ORUtils::MemoryBlock<Vector3s> block_positions;
	DECLARE_ATOMIC(unsigned int, current_fill_index);
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyAnalysisFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	static std::vector<int> GetAllocatedHashCodes(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<int>();
	}

	static std::vector<int> GetUtilizedHashCodes(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<int>();
	}


	static std::vector<Vector3s> GetAllocatedBlockPositions(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		std::vector<Vector3s> pos_vector = {TO_SHORT3(volume->index.GetVolumeOffset())};
		return pos_vector;
	}

	static std::vector<Vector3s> GetUtilizedBlockPositions(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		std::vector<Vector3s> pos_vector = {TO_SHORT3(volume->index.GetVolumeOffset())};
		return pos_vector;
	}

	static std::vector<Vector3s> GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<Vector3s>();
	}

	static int ComputeAllocatedHashBlockCount(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return 1;
	}

	static int ComputeUtilizedHashBlockCount(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return 1;
	}

	static unsigned int CountHashBlocksWithDepthWeightInRange(const VoxelVolume<TVoxel, PlainVoxelArray>* volume, Extent2Di range) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot count hash blocks with a specific property within a volume indexed by a plain voxel array.");
		return 0;
	}
};
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyAnalysisFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	static std::vector<int> GetAllocatedHashCodes(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		unsigned int allocated_count = ComputeAllocatedHashBlockCount(volume);
		AllocatedHashesAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithIndex(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<Vector3s> GetAllocatedBlockPositions(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		unsigned int allocated_count = ComputeAllocatedHashBlockCount(volume);
		if (allocated_count == 0u) {
			return std::vector<Vector3s>();
		}
		BlockPositionAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithIndex(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<int> GetUtilizedHashCodes(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		const int* codes = volume->index.GetUtilizedBlockHashCodes();
		return raw_block_to_std_vector(codes, TMemoryDeviceType, volume->index.GetUtilizedBlockCount());
	}

	static std::vector<Vector3s> GetUtilizedBlockPositions(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		unsigned int utilized_count = volume->index.GetUtilizedBlockCount();
		if (utilized_count == 0u) {
			return std::vector<Vector3s>();
		}
		BlockPositionAggregationFunctor<TMemoryDeviceType> aggregator_functor(utilized_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseUtilizedWithIndex(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<Vector3s> GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		std::vector<Vector3s> allocated_positions = GetAllocatedBlockPositions(volume);
		std::unordered_set<Vector3s> allocated_positions_set(allocated_positions.begin(), allocated_positions.end());
		std::vector<Vector3s> utilized_positions = GetUtilizedBlockPositions(volume);
		std::unordered_set<Vector3s> utilized_positions_set(utilized_positions.begin(), utilized_positions.end());
		std::unordered_set<Vector3s> difference_set = allocated_positions_set - utilized_positions_set;
		std::vector<Vector3s> difference_vector(difference_set.begin(), difference_set.end());
		return difference_vector;
	}

	static unsigned int ComputeAllocatedHashBlockCount(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		AllocatedHashBlockCountFunctor<TMemoryDeviceType> count_functor;
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithIndex(volume->index, count_functor);
		return count_functor.get_count();
	}

	static unsigned int ComputeUtilizedHashBlockCount(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		return volume->index.GetUtilizedBlockCount();
	}

	static unsigned int CountHashBlocksWithDepthWeightInRange(const VoxelVolume<TVoxel, VoxelBlockHash>* volume, Extent2Di range) {
		typedef RetrieveIsVoxelInDepthWeightRange<TVoxel, unsigned int, TVoxel::hasWeightInformation> RetrievalFunctorType;
		typedef ReduceBinAndFunctor<TVoxel, VoxelBlockHash, unsigned int> BlockReduceFunctorType;
		typedef ReduceSumFunctor<TVoxel, VoxelBlockHash, unsigned int> ResultReduceFunctorType;
		RetrievalFunctorType functor{range};
		Vector3i position;

		return BlockVolumeReductionEngine<TVoxel, TMemoryDeviceType>::
		template ReduceUtilizedBlocks<BlockReduceFunctorType, ResultReduceFunctorType, RetrievalFunctorType, unsigned int>
				(position, volume, functor);
	}
};
//endregion

//region======================= VOLUME BOUNDS COMPUTATION =============================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelBoundsFunctor;

template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> {
	static Vector6i Compute(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

		Vector6i bounds = Vector6i(0);
		bounds.min_x = std::numeric_limits<int>::max();
		bounds.min_y = std::numeric_limits<int>::max();
		bounds.min_z = std::numeric_limits<int>::max();

		const TVoxel* voxel_blocks = volume->GetVoxels();
		const HashEntry* hash_table = volume->index.GetEntries();
		int hash_entry_count = volume->index.hash_entry_count;

		//TODO: if OpenMP standard is 3.1 or above, use OpenMP parallel for reduction clause with (max:maxVoxelPointX,...) -Greg (GitHub: Algomorph)
		for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {

			const HashEntry& hash_entry = hash_table[hash_code];

			if (hash_entry.ptr < 0) continue;

			//position of the current entry in 3D space
			Vector3i hash_block_min_voxels = hash_entry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hash_block_max_voxels =
					(hash_entry.pos.toInt() + Vector3i(1, 1, 1)) * VOXEL_BLOCK_SIZE;

			if (bounds.min_x > hash_block_min_voxels.x) {
				bounds.min_x = hash_block_min_voxels.x;
			}
			if (bounds.max_x < hash_block_max_voxels.x) {
				bounds.max_x = hash_block_max_voxels.x;
			}
			if (bounds.min_y > hash_block_min_voxels.y) {
				bounds.min_y = hash_block_min_voxels.y;
			}
			if (bounds.max_y < hash_block_max_voxels.y) {
				bounds.max_y = hash_block_max_voxels.y;
			}
			if (bounds.min_z > hash_block_min_voxels.z) {
				bounds.min_z = hash_block_min_voxels.z;
			}
			if (bounds.max_z < hash_block_max_voxels.z) {
				bounds.max_z = hash_block_max_voxels.z;
			}
		}
		return bounds;
	}
};
template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> {
	static Vector6i Compute(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		Vector3i offset = volume->index.GetVolumeOffset();
		Vector3i size = volume->index.GetVolumeSize();
		return {offset.x, offset.y, offset.z,
		        offset.x + size.x, offset.y + size.y, offset.z + size.z};
	}
};
#ifdef __CUDACC__
namespace {
// CUDA kernel implementations

__global__ void computeVoxelBounds(const HashEntry* hash_table, Vector6i* bounds, int hash_entry_count) {
	int hash_code = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash_code >= hash_entry_count) return;

	const HashEntry& hashEntry = hash_table[hash_code];
	if (hashEntry.ptr < 0) return;

	Vector3i hashEntryPosVoxels = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

	atomicMin(&(bounds->min_x), hashEntryPosVoxels.x);
	atomicMin(&(bounds->min_y), hashEntryPosVoxels.y);
	atomicMin(&(bounds->min_z), hashEntryPosVoxels.z);
	atomicMax(&(bounds->max_x), hashEntryPosVoxels.x + VOXEL_BLOCK_SIZE);
	atomicMax(&(bounds->max_y), hashEntryPosVoxels.y + VOXEL_BLOCK_SIZE);
	atomicMax(&(bounds->max_z), hashEntryPosVoxels.z + VOXEL_BLOCK_SIZE);
}
} // end anonymous namespace


template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> {
	static Vector6i Compute(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

		Vector6i bounds = Vector6i(0);
		bounds.min_x = std::numeric_limits<int>::max();
		bounds.min_y = std::numeric_limits<int>::max();
		bounds.min_z = std::numeric_limits<int>::max();

		const TVoxel* voxels = volume->GetVoxels();
		const HashEntry* hashTable = volume->index.GetEntries();
		int noTotalEntries = volume->index.hash_entry_count;

		dim3 cudaBlockSize(256, 1);
		dim3 cudaGridSize((int) ceil((float) noTotalEntries / (float) cudaBlockSize.x));

		Vector6i* boundsCuda = nullptr;

		ORcudaSafeCall(cudaMalloc((void**) &boundsCuda, sizeof(Vector6i)));
		ORcudaSafeCall(cudaMemcpy(boundsCuda, (void*) &bounds, sizeof(Vector6i), cudaMemcpyHostToDevice));

		computeVoxelBounds <<< cudaGridSize, cudaBlockSize >>> (hashTable, boundsCuda, noTotalEntries);
		ORcudaKernelCheck;

		ORcudaSafeCall(cudaMemcpy((void*) &bounds, boundsCuda, sizeof(Vector6i), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(boundsCuda));
		return bounds;
	}
};

template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> {
	static Vector6i Compute(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		const PlainVoxelArray::IndexData* indexData = volume->index.GetIndexData();
		return Vector6i(indexData->offset.x, indexData->offset.y, indexData->offset.z,
						indexData->offset.x + indexData->size.x, indexData->offset.y + indexData->size.y,
						indexData->offset.z + indexData->size.z);
	}
};

#endif

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, typename TStaticPredicateFunctor>
struct ComputeConditionalVoxelBoundsFunctor {
	ComputeConditionalVoxelBoundsFunctor() {
		INITIALIZE_ATOMIC(int, min_x, INT_MAX);
		INITIALIZE_ATOMIC(int, min_y, INT_MAX);
		INITIALIZE_ATOMIC(int, min_z, INT_MAX);
		INITIALIZE_ATOMIC(int, max_x, INT_MIN);
		INITIALIZE_ATOMIC(int, max_y, INT_MIN);
		INITIALIZE_ATOMIC(int, max_z, INT_MIN);
	}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const TVoxel& voxel, const Vector3i& voxel_position) {
		if (TStaticPredicateFunctor::isSatisfiedBy(voxel)) {
			ATOMIC_MIN(min_x, voxel_position.x);
			ATOMIC_MIN(min_y, voxel_position.y);
			ATOMIC_MIN(min_z, voxel_position.z);
			ATOMIC_MAX(max_x, voxel_position.x);
			ATOMIC_MAX(max_y, voxel_position.y);
			ATOMIC_MAX(max_z, voxel_position.z);
		}
	}

	~ComputeConditionalVoxelBoundsFunctor() {
		CLEAN_UP_ATOMIC(min_x);CLEAN_UP_ATOMIC(min_y);CLEAN_UP_ATOMIC(min_z);CLEAN_UP_ATOMIC(max_x);CLEAN_UP_ATOMIC(
				max_y);CLEAN_UP_ATOMIC(max_z);
	}

	Extent3Di GetBounds() {
		return {
				GET_ATOMIC_VALUE_CPU(min_x),
				GET_ATOMIC_VALUE_CPU(min_y),
				GET_ATOMIC_VALUE_CPU(min_z),
				GET_ATOMIC_VALUE_CPU(max_x) + 1,
				GET_ATOMIC_VALUE_CPU(max_y) + 1,
				GET_ATOMIC_VALUE_CPU(max_z) + 1
		};
	}

private:

	DECLARE_ATOMIC(int, min_x);
	DECLARE_ATOMIC(int, min_y);
	DECLARE_ATOMIC(int, min_z);
	DECLARE_ATOMIC(int, max_x);
	DECLARE_ATOMIC(int, max_y);
	DECLARE_ATOMIC(int, max_z);
};
//endregion ===============================================================================================
// region ===================================== HISTOGRAMS ================================================
template<typename TWarp, MemoryDeviceType TMemoryDeviceType, bool THasWarpUpdate>
struct WarpHistogramFunctor;

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct WarpHistogramFunctor<TWarp, TMemoryDeviceType, false> {
	explicit WarpHistogramFunctor(Histogram& histogram, float maximum) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot compute warp update length histogram on voxels without warp updates, aborting.");
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TWarp& warp) {}
};

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct WarpHistogramFunctor<TWarp, TMemoryDeviceType, true> {
	explicit WarpHistogramFunctor(Histogram& histogram, float maximum) :
			max_warp_update_length(maximum),
			histogram_bin_count(histogram.GetBinCount()),
			bins_device(histogram.GetBinData()),
			unit_count_device(histogram.GetUnitCountData()) {
	}

	const int histogram_bin_count;
	const float max_warp_update_length;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const TWarp& warp) {
		float warp_update_length = ORUtils::length(warp.warp_update);
		int bin_index = 0;

		if (max_warp_update_length > 0.0f) {
			bin_index = ORUTILS_MIN(histogram_bin_count - 1, (int) (warp_update_length * histogram_bin_count / max_warp_update_length));
		}
#ifdef __CUDACC__
		atomicAdd(bins_device + bin_index, 1);
		atomicAdd(unit_count_device, 1u);
#else
#pragma omp critical
		{
			bins_device[bin_index]++;
			(*unit_count_device)++;
		}
#endif
	}

private:
	unsigned int* bins_device;
	unsigned long long int* unit_count_device;
};
// endregion ==============================================================================================