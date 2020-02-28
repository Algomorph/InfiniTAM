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
#include "../../../Utils/CUDAUtils.h"
#include "../../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h"
#include "../../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#endif

#include "../../../Engines/Traversal/Interface/HashTableTraversal.h"
#include "../../../Engines/Reduction/Interface/VolumeReduction.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../Configuration.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../../WarpType.h"
#include "../Statistics.h"
#include "../../MemoryBlock_StdVector_Converter.h"
#include "../../../Engines/Common/WarpAccessFunctors.h"


using namespace ITMLib;

// region =========================================== ATOMIC VECTOR/GRADIENT FIELD MIN/MEAN/MAX ========================

template<typename TVoxel, Statistic TStatistic, WarpType TWarpType>
struct HandleVectorLengthAggregate;
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));

	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_FRAMEWISE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
		ATOMIC_ADD(count, 1u);
	}
};

template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));

	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_UPDATE> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));

	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_GRADIENT0> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MINIMUM, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MAXIMUM, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
	}
};
template<typename TVoxel>
struct HandleVectorLengthAggregate<TVoxel, MEAN, WARP_GRADIENT1> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.warp_update)));
		ATOMIC_ADD(count, 1u);
	}
};

template<bool hasWarp, typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor;

template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor<false, TVoxel, TIndex, TDeviceType, TStatistic, TWarpType> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION("Voxels need to have warp information to get warp statistics.");
		return 0;
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic, WarpType TWarpType>
struct ComputeWarpLengthStatisticFunctor<true, TVoxel, TIndex, TDeviceType, TStatistic, TWarpType> {

	~ComputeWarpLengthStatisticFunctor() {
		CLEAN_UP_ATOMIC(aggregate);CLEAN_UP_ATOMIC(count);
	}

	static double compute(VoxelVolume<TVoxel, TIndex>* scene) {
		ComputeWarpLengthStatisticFunctor instance;
		INITIALIZE_ATOMIC(double, instance.aggregate, 0.0);
		INITIALIZE_ATOMIC(unsigned int, instance.count, 0u);
		VolumeTraversalEngine<TVoxel, TIndex, TDeviceType>::TraverseUtilized(scene, instance);
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
	void operator()(TVoxel& voxel) {
		HandleVectorLengthAggregate<TVoxel, TStatistic, TWarpType>::aggregateStatistic(aggregate, count, voxel);
	}
};

//endregion
// region =========================================== REDUCTION VECTOR/GRADIENT FIELD MIN/MEAN/MAX ========================

template<typename TVoxel, ITMLib::WarpType TWarpType>
struct RetreiveWarpLengthFunctor {
	_CPU_AND_GPU_CODE_
	inline static float retrieve(const TVoxel& voxel) {
		return ORUtils::length(WarpAccessStaticFunctor<TVoxel, TWarpType>::GetWarp(voxel));
	}
};

template<typename TVoxel, typename TIndex, ITMLib::WarpType TWarpType, ITMLib::Statistic TStatistic>
struct ReduceWarpLengthStatisticFunctor;

template<typename TVoxel, typename TIndex>
struct ReduceWarpLengthStatisticFunctor<TVoxel, TIndex, ITMLib::WARP_UPDATE, ITMLib::MAXIMUM> {
public:
	_CPU_AND_GPU_CODE_
	inline static const ReductionResult<float, TIndex>& reduce(
			const ReductionResult<float, TIndex>& item1, const ReductionResult<float, TIndex>& item2) {
		return (item1.value > item2.value) ? item1 : item2;
	}
};

// endregion

//region ================== **** VOXEL COUNTS **** =====================================================================

//region =========================== COUNT ALL VOXELS ==================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	inline
	static unsigned int compute(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		PlainVoxelArray& index = volume->index;
		return static_cast<unsigned int>(index.GetVolumeSize().x * index.GetVolumeSize().y * index.GetVolumeSize().z);
	}
};
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyStatisticsFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	inline
	static unsigned int compute(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		return static_cast<unsigned int>(HashOnlyStatisticsFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType>
		::ComputeAllocatedHashBlockCount(volume)) * VOXEL_BLOCK_SIZE3;
	}
};
//endregion
//region ============================================ COUNT VOXELS WITH SPECIFIC SDF VALUE =============================

template<bool hasSDFInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificValue;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificValue<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit ComputeVoxelCountWithSpecificValue(float value) : value(value) {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	}

	~ComputeVoxelCountWithSpecificValue() {
		CLEAN_UP_ATOMIC(count);
	}

	static int compute(VoxelVolume<TVoxel, TIndex>* scene, float value) {
		ComputeVoxelCountWithSpecificValue instance(value);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAll(scene, instance);
		return GET_ATOMIC_VALUE_CPU(instance.count);
	}

	DECLARE_ATOMIC(unsigned int, count);
	float value = 0.0f;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		if (TVoxel::valueToFloat(voxel.sdf) == value) {
			ATOMIC_ADD(count, 1u);
		}
	}
};

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificValue<false, TVoxel, TIndex, TMemoryDeviceType> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene, float value) {
		DIEWITHEXCEPTION(
				"Voxel volume issued to count voxels with specific SDF value appears to have no sdf information. "
				"Voxels in volume need to have sdf information for this operation to work.");
		return 0;
	}
};
//endregion
//region =========================================== COUNT VOXELS WITH SPECIFIC FLAGS ==================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificFlags;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificFlags<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit ComputeVoxelCountWithSpecificFlags(VoxelFlags flags) : flags(flags) {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	}

	~ComputeVoxelCountWithSpecificFlags() {
		CLEAN_UP_ATOMIC(count);
	}

	static int compute(VoxelVolume<TVoxel, TIndex>* scene, VoxelFlags flags) {
		ComputeVoxelCountWithSpecificFlags instance(flags);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAll(scene, instance);
		return GET_ATOMIC_VALUE_CPU(instance.count);
	}

	DECLARE_ATOMIC(unsigned int, count);
	VoxelFlags flags;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		if (voxel.flags == flags) {
			ATOMIC_ADD(count, 1u);
		}
	}
};

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificFlags<false, TVoxel, TIndex, TMemoryDeviceType> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene, float value) {
		DIEWITHEXCEPTION(
				"Voxel volume issued to count voxels with specific semantic flags appears to have no semantic information. "
				"Voxels in volume need to have semantic flags field for this operation to work.");
		return 0;
	}
};
//endregion
//region ================================== COUNT ALTERED VOXELS =======================================================

template<typename TVoxel>
struct IsAlteredCountFunctor {
	IsAlteredCountFunctor() {
		INITIALIZE_ATOMIC(unsigned int, count, 0u);
	};

	~IsAlteredCountFunctor() {
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
//endregion
//region =========================================== SUM OF TOTAL SDF ==================================================

template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {

	static double compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Voxels need to have semantic information to be marked as truncated or non-truncated.");
		return 0.0;
	}
};
template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct SumSDFFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit SumSDFFunctor(ITMLib::VoxelFlags voxelType) : voxelType(voxelType) {
		INITIALIZE_ATOMIC(double, sum, 0.0);
	}

	~SumSDFFunctor() {
		CLEAN_UP_ATOMIC(sum);
	}

	static double compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		SumSDFFunctor instance(voxelType);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAll(scene, instance);
		return GET_ATOMIC_VALUE_CPU(instance.sum);
	}

	DECLARE_ATOMIC(double, sum);
	ITMLib::VoxelFlags voxelType;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		if (voxelType == (ITMLib::VoxelFlags) voxel.flags) {
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

	static Extent3Di compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		DIEWITHEXCEPTION_REPORTLOCATION(
				"Voxels need to have semantic information to be marked as truncated or non-truncated.");
		return {};
	}
};
template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor<true, TVoxel, TIndex, TMemoryDeviceType> {
	explicit FlagMatchBBoxFunctor(ITMLib::VoxelFlags voxelType) : voxelType(voxelType) {
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


	static Extent3Di compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		FlagMatchBBoxFunctor instance(voxelType);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(scene, instance);
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
	ITMLib::VoxelFlags voxelType;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel, Vector3i voxelPosition) {
		if (voxelType == (ITMLib::VoxelFlags) voxel.flags) {
			ATOMIC_MIN(min_x, voxelPosition.x);
			ATOMIC_MIN(min_y, voxelPosition.y);
			ATOMIC_MIN(min_z, voxelPosition.z);
			ATOMIC_MAX(max_x, voxelPosition.x);
			ATOMIC_MAX(max_y, voxelPosition.y);
			ATOMIC_MAX(max_z, voxelPosition.z);
		}
	}
};
//endregion
//region ===================================== HASH BLOCK STATS ========================================================

template<MemoryDeviceType TMemoryDeviceType>
struct AllocatedHashBlockCountFunctor {
	AllocatedHashBlockCountFunctor() {
		INITIALIZE_ATOMIC(int, allocated_hash_block_count, 0);
	}

	~AllocatedHashBlockCountFunctor() {
		CLEAN_UP_ATOMIC(allocated_hash_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code) {
		if (entry.ptr >= 0) {
			ATOMIC_ADD(allocated_hash_block_count, 1);
		}
	}

	int get_count() {
		return GET_ATOMIC_VALUE_CPU(allocated_hash_block_count);
	}

private:
	DECLARE_ATOMIC(int, allocated_hash_block_count);
};

template<MemoryDeviceType TMemoryDeviceType>
struct AllocatedHashesAggregationFunctor {
	AllocatedHashesAggregationFunctor(int allocated_block_count) : hash_codes(allocated_block_count,
	                                                                          TMemoryDeviceType) {
		INITIALIZE_ATOMIC(int, current_fill_index, 0);
		hash_codes_device = hash_codes.GetData(TMemoryDeviceType);
	}


	~AllocatedHashesAggregationFunctor() {
		CLEAN_UP_ATOMIC(current_fill_index);
	}


	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code) {
		if (entry.ptr >= 0) {
			int index = ATOMIC_ADD(current_fill_index, 1);
			hash_codes_device[index] = hash_code;
		}
	}

	std::vector<int> data() {
		return ORUtils_MemoryBlock_to_std_vector(hash_codes, TMemoryDeviceType);
	}

private:
	int* hash_codes_device;
	ORUtils::MemoryBlock<int> hash_codes;
	DECLARE_ATOMIC(int, current_fill_index);
};


template<MemoryDeviceType TMemoryDeviceType>
struct BlockPositionAggregationFunctor {
	BlockPositionAggregationFunctor(int block_count) : block_positions(block_count,
	                                                                   TMemoryDeviceType) {
		INITIALIZE_ATOMIC(int, current_fill_index, 0);
		block_positions_device = block_positions.GetData(TMemoryDeviceType);
	}


	~BlockPositionAggregationFunctor() {
		CLEAN_UP_ATOMIC(current_fill_index);
	}


	_DEVICE_WHEN_AVAILABLE_
	void operator()(HashEntry& entry, int hash_code) {
		if (entry.ptr >= 0) {
			int index = ATOMIC_ADD(current_fill_index, 1);
			block_positions_device[index] = entry.pos;
		}
	}

	std::vector<Vector3s> data() {
		return ORUtils_MemoryBlock_to_std_vector(block_positions, TMemoryDeviceType);
	}

private:
	Vector3s* block_positions_device;
	ORUtils::MemoryBlock<Vector3s> block_positions;
	DECLARE_ATOMIC(int, current_fill_index);
};

//template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
//struct HashOnlyStatisticsFunctor;
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyStatisticsFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	static std::vector<int> GetAllocatedHashCodes(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<int>();
	}

	static std::vector<int> GetUtilizedHashCodes(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<int>();
	}


	static std::vector<Vector3s> GetAllocatedBlockPositions(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		std::vector<Vector3s> pos_vector = {TO_SHORT3(volume->index.GetVolumeOffset())};
		return pos_vector;
	}

	static std::vector<Vector3s> GetUtilizedBlockPositions(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		std::vector<Vector3s> pos_vector = {TO_SHORT3(volume->index.GetVolumeOffset())};
		return pos_vector;
	}

	static int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return 0;
	}
};
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyStatisticsFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	static std::vector<int> GetAllocatedHashCodes(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		int allocated_count = ComputeAllocatedHashBlockCount(volume);
		AllocatedHashesAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<Vector3s> GetAllocatedBlockPositions(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		int allocated_count = ComputeAllocatedHashBlockCount(volume);
		BlockPositionAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<int> GetUtilizedHashCodes(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		const int* codes = volume->index.GetUtilizedBlockHashCodes();
		return raw_block_to_std_vector(codes, TMemoryDeviceType, volume->index.GetUtilizedHashBlockCount());
	}

	static std::vector<Vector3s> GetUtilizedBlockPositions(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		int allocated_count = volume->index.GetUtilizedHashBlockCount();
		BlockPositionAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseUtilizedWithHashCode(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		AllocatedHashBlockCountFunctor<TMemoryDeviceType> count_functor;
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseAllWithHashCode(volume->index, count_functor);
		return count_functor.get_count();
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

		const TVoxel* voxel_blocks = volume->voxels.GetVoxelBlocks();
		const HashEntry* hash_table = volume->index.GetEntries();
		int hash_entry_count = volume->index.hashEntryCount;

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
	int hash = threadIdx.x + blockIdx.x * blockDim.x;
	if (hash >= hash_entry_count) return;

	const HashEntry& hashEntry = hash_table[hash];
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

		const TVoxel* voxelBlocks = volume->voxels.GetVoxelBlocks();
		const HashEntry* hashTable = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;

		dim3 cudaBlockSize(256, 1);
		dim3 cudaGridSize((int) ceil((float) noTotalEntries / (float) cudaBlockSize.x));

		Vector6i* boundsCuda = nullptr;

		ORcudaSafeCall(cudaMalloc((void**) &boundsCuda, sizeof(Vector6i)));
		ORcudaSafeCall(cudaMemcpy(boundsCuda, (void*) &bounds, sizeof(Vector6i), cudaMemcpyHostToDevice));

		computeVoxelBounds << < cudaGridSize, cudaBlockSize >> > (hashTable, boundsCuda, noTotalEntries);
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
//endregion