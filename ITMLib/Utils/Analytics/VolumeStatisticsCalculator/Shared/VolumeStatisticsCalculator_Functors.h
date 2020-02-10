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
#include "../../../../Utils/CUDAUtils.h"
#include "../../../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h"
#include "../../../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#endif

#include "../../../../Engines/Traversal/Interface/HashTableTraversal.h"
#include "../../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../../ORUtils/PlatformIndependence.h"
#include "../../../Configuration.h"
#include "../../../../Objects/Volume/VoxelVolume.h"
#include "../../../../Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"


using namespace ITMLib;

namespace ITMLib {
enum Statistic {
	MINIMUM,
	MAXIMUM,
	MEAN
};
}

//================================================ VECTOR/GRADIENT FIELDS ==============================================

template<bool hasCumulativeWarp, typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic>
struct ComputeFramewiseWarpLengthStatisticFunctor;

template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic>
struct ComputeFramewiseWarpLengthStatisticFunctor<false, TVoxel, TIndex, TDeviceType, TStatistic> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION("Voxels need to have flow warp information to get flow warp statistics.");
		return 0;
	}
};

template<typename TVoxel, Statistic TStatistic>
struct HandleAggregate;
template<typename TVoxel>
struct HandleAggregate<TVoxel, MINIMUM> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));

	}
};
template<typename TVoxel>
struct HandleAggregate<TVoxel, MAXIMUM> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
	}
};
template<typename TVoxel>
struct HandleAggregate<TVoxel, MEAN> {
	_DEVICE_WHEN_AVAILABLE_
	inline static void
	aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel) {
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.framewise_warp)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel, typename TIndex, MemoryDeviceType TDeviceType, Statistic TStatistic>
struct ComputeFramewiseWarpLengthStatisticFunctor<true, TVoxel, TIndex, TDeviceType, TStatistic> {

	~ComputeFramewiseWarpLengthStatisticFunctor() {
		CLEAN_UP_ATOMIC(aggregate);CLEAN_UP_ATOMIC(count);
	}

	static double compute(VoxelVolume<TVoxel, TIndex>* scene) {
		ComputeFramewiseWarpLengthStatisticFunctor instance;
		INITIALIZE_ATOMIC(double, instance.aggregate, 0.0);
		INITIALIZE_ATOMIC(unsigned int, instance.count, 0u);
		VolumeTraversalEngine<TVoxel, TIndex, TDeviceType>::VoxelTraversal(scene, instance);
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

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		HandleAggregate<TVoxel, TStatistic>::aggregateStatistic(aggregate, count, voxel);
	}
};

//================================================== COUNT VOXELS WITH SPECIFIC VALUE ==================================

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
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelTraversal(scene, instance);
		return GET_ATOMIC_VALUE_CPU(instance.count);
	}

	DECLARE_ATOMIC(unsigned int, count);
	float value = 0.0f;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		ATOMIC_ADD(count, (unsigned int) (TVoxel::valueToFloat(voxel.sdf) == value));
	}
};

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeVoxelCountWithSpecificValue<false, TVoxel, TIndex, TMemoryDeviceType> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene, float value) {
		DIEWITHEXCEPTION(
				"Voxel volume issued to count voxels with specific SDF value appears to have no sdf information. "
				"Voxels need to have sdf information.");
		return 0;
	}
};


//================================================== SUM OF TOTAL SDF ==================================================

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
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelTraversal(scene, instance);
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

//========================================= COUNT ALTERED VOXELS =======================================================

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

//============================================ BOUNDING BOX COMPUTATIONS ===============================================
template<bool hasSemanticInformation, typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor;

template<class TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct FlagMatchBBoxFunctor<false, TVoxel, TIndex, TMemoryDeviceType> {

	static Extent3D compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
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


	static Extent3D compute(VoxelVolume<TVoxel, TIndex>* scene, ITMLib::VoxelFlags voxelType) {
		FlagMatchBBoxFunctor instance(voxelType);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelPositionTraversal(scene, instance);
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

//============================================ HASH BLOCK STATS ========================================================

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


template<typename T>
inline std::vector<T> ORUtils_MemoryBlock_to_std_vector(const ORUtils::MemoryBlock<T>& block, MemoryDeviceType device_type) {
	std::vector<T> vector(block.dataSize);
	if (device_type == MEMORYDEVICE_CPU) {
		memcpy(vector.data(), block.GetData(MEMORYDEVICE_CPU), vector.size() * sizeof(T));
	} else {
#ifndef COMPILE_WITHOUT_CUDA
		ORcudaSafeCall(cudaMemcpy(vector.data(), block.GetData(MEMORYDEVICE_CUDA), vector.size() * sizeof(T),
		                          cudaMemcpyDeviceToHost));
#else
		DIEWITHEXCEPTION_REPORTLOCATION("Not supported without compilation with CUDA.");
#endif
	}
	return vector;
}

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
struct AllocatedBlockPositionAggregationFunctor {
	AllocatedBlockPositionAggregationFunctor(int allocated_block_count) : block_positions(allocated_block_count,
	                                                                                      TMemoryDeviceType) {
		INITIALIZE_ATOMIC(int, current_fill_index, 0);
		block_positions_device = block_positions.GetData(TMemoryDeviceType);
	}


	~AllocatedBlockPositionAggregationFunctor() {
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

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyStatisticsFunctor;
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct HashOnlyStatisticsFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	static std::vector<int> GetAllocatedHashCodes(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		return std::vector<int>();
	}

	static std::vector<Vector3s> GetAllocatedBlockPositions(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
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
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseWithHashCode(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static std::vector<Vector3s> GetAllocatedBlockPositions(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		int allocated_count = ComputeAllocatedHashBlockCount(volume);
		AllocatedBlockPositionAggregationFunctor<TMemoryDeviceType> aggregator_functor(allocated_count);
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseWithHashCode(volume->index, aggregator_functor);
		return aggregator_functor.data();
	}

	static int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		AllocatedHashBlockCountFunctor<TMemoryDeviceType> count_functor;
		HashTableTraversalEngine<TMemoryDeviceType>::TraverseWithHashCode(volume->index, count_functor);
		return count_functor.get_count();
	}
};
//================================== COUNT ALL VOXELS ==================================================================
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor;

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, PlainVoxelArray, TMemoryDeviceType> {
	inline
	static int compute(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		PlainVoxelArray& index = volume->index;
		return index.GetVolumeSize().x * index.GetVolumeSize().y * index.GetVolumeSize().z;
	}
};

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType> {
	inline
	static int compute(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {
		return HashOnlyStatisticsFunctor<TVoxel, VoxelBlockHash, TMemoryDeviceType>::ComputeAllocatedHashBlockCount(volume) * VOXEL_BLOCK_SIZE3;
	}
};