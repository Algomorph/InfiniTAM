//  ================================================================
//  Created by Gregory Kramida on 1/31/20.
//  Copyright (c) 2020 Gregory Kramida
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

#include "../Interface/TwoVolumeTraversal.h"
#include "TwoVolumeTraversal_CUDA_VoxelBlockHash_Kernels.cuh"
#include "../../../Utils/Enums/ExecutionMode.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Utils/CUDA/CudaCallWrappers.cuh"

namespace ITMLib {


template<typename TVoxel1, typename TVoxel2>
class TwoVolumeTraversalEngine<TVoxel1, TVoxel2, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CUDA> {
private:

	template<typename TFunctor, typename TDeviceTraversalFunction>
	inline static void
	TraverseAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, TDeviceTraversalFunction&& device_traversal_function) {
		TVoxel1* voxels1 = volume1->GetVoxels();
		TVoxel2* voxels2 = volume2->GetVoxels();
		const HashEntry* hash_table1 = volume1->index.GetIndexData();
		const HashEntry* hash_table2 = volume2->index.GetIndexData();
		const int hash_entry_count = volume1->index.hash_entry_count;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 block_voxel_per_thread_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_grid_size(hash_entry_count);

		std::forward<TDeviceTraversalFunction>(device_traversal_function)(
				hash_per_block_grid_size, block_voxel_per_thread_block_size,
				voxels1, voxels2, hash_table1, hash_table2, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TFunctor, typename TDeviceTraversalFunction>
	inline static void
	TraverseUtilized_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {
		TVoxel1* voxels1 = volume1->GetVoxels();
		TVoxel2* voxels2 = volume2->GetVoxels();
		const HashEntry* hash_table1 = volume1->index.GetIndexData();
		const HashEntry* hash_table2 = volume2->index.GetIndexData();
		const int utilized_entry_count = volume1->index.GetUtilizedBlockCount();
		const int* utilized_hash_codes = volume1->index.GetUtilizedBlockHashCodes();

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 block_voxel_per_thread_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 hash_per_block_grid_size(utilized_entry_count);

		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(
				hash_per_block_grid_size, block_voxel_per_thread_block_size,
				voxels1, voxels2, hash_table1, hash_table2, utilized_hash_codes, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<bool TPassPositionToFunctor,
			typename TBooleanFunctor,
			typename TVoxelPredicate1Functor,
			typename TVoxelPredicate2Functor>
	inline static bool
	TraverseAndCompareAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor,
			TVoxelPredicate1Functor&& voxel1_qualifies_for_comparison,
			TVoxelPredicate2Functor&& voxel2_qualifies_for_comparison) {
		// assumes functor is allocated in main memory

		int hash_entry_count = volume1->index.hash_entry_count;

		// allocate intermediate-result buffers for use on the CUDA in subsequent routine calls
		ORUtils::MemoryBlock<bool> mismatch_encountered(1, true, true);
		*mismatch_encountered.GetData(MEMORYDEVICE_CPU) = false;
		mismatch_encountered.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashMatchInfo> hash_match_info(1, true, true);
		hash_match_info.GetData(MEMORYDEVICE_CPU)->matched_hash_count = 0;
		hash_match_info.GetData(MEMORYDEVICE_CPU)->unmatched_hash_count = 0;
		hash_match_info.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashPair> matched_hash_code_pairs(hash_entry_count, true, true);
		ORUtils::MemoryBlock<UnmatchedHash> unmatched_hash_blocks(hash_entry_count, true, true);

		// these will be needed for various matching & traversal operations
		TVoxel1* voxels1 = volume1->GetVoxels();
		TVoxel2* voxels2 = volume2->GetVoxels();
		const HashEntry* hash_table1 = volume1->index.GetIndexData();
		const HashEntry* hash_table2 = volume2->index.GetIndexData();

		dim3 cuda_block_size_hash_block_per_thread(256, 1);
		dim3 grid_size_multiple_hash_blocks(static_cast<int>(ceil(static_cast<float>(hash_entry_count) /
		                                                          static_cast<float>(cuda_block_size_hash_block_per_thread.x))));
		matchUpHashEntriesByPosition
		<<< grid_size_multiple_hash_blocks, cuda_block_size_hash_block_per_thread >>>
				(hash_table1, hash_table2, hash_entry_count,
				 matched_hash_code_pairs.GetData(MEMORYDEVICE_CUDA),
				 unmatched_hash_blocks.GetData(MEMORYDEVICE_CUDA),
				 hash_match_info.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// check unmatched hashes
		hash_match_info.UpdateHostFromDevice();
		dim3 block_voxel_per_thread_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		int unmatchedHashCount = hash_match_info.GetData(MEMORYDEVICE_CPU)->unmatched_hash_count;
		if (unmatchedHashCount > 0) {
			dim3 grid_size_unmatched_blocks(unmatchedHashCount);
			checkIfUnmatchedVoxelBlocksAreAltered
			<<< grid_size_unmatched_blocks, block_voxel_per_thread_block_size >>>
					(voxels1, voxels2,
					 hash_table1, hash_table2,
					 unmatched_hash_blocks.GetData(MEMORYDEVICE_CUDA),
					 hash_match_info.GetData(MEMORYDEVICE_CUDA),
					 mismatch_encountered.GetData(MEMORYDEVICE_CUDA),
					 voxel1_qualifies_for_comparison, voxel2_qualifies_for_comparison);
			ORcudaKernelCheck;
		}

		// if an unmatched block in either volume was altered, return false
		mismatch_encountered.UpdateHostFromDevice();
		if (*mismatch_encountered.GetData(MEMORYDEVICE_CPU)) return false;

		int matched_hash_count = hash_match_info.GetData(MEMORYDEVICE_CPU)->matched_hash_count;

		if (matched_hash_count == 0) return true;


		// perform voxel traversal on the CUDA, matching individual voxels at corresponding locations
		dim3 grid_size_matched_blocks(matched_hash_count);

		internal::CallCUDAonUploadedFunctor(
				functor,
				[&](TBooleanFunctor* functor_device){
					checkCorrespondingHashBlocks<TPassPositionToFunctor>
					<<< grid_size_matched_blocks, block_voxel_per_thread_block_size >>>
							(voxels1, voxels2, hash_table1, hash_table2,
							 matched_hash_code_pairs.GetData(MEMORYDEVICE_CUDA),
							 hash_match_info.GetData(MEMORYDEVICE_CUDA),
							 functor_device, mismatch_encountered.GetData(MEMORYDEVICE_CUDA),
							 voxel1_qualifies_for_comparison, voxel2_qualifies_for_comparison);
				}
		);

		mismatch_encountered.UpdateHostFromDevice();
		return !(*mismatch_encountered.GetData(MEMORYDEVICE_CPU));
	}

public:
	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {

		auto return_true1 = [] __device__(const TVoxel1& voxel1) { return true; };
		auto return_true2 = [] __device__(const TVoxel2& voxel2) { return true; };

		return TraverseAndCompareAll_Generic<false>(volume1, volume2, functor, return_true1, return_true2);
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {
		auto return_true1 = [] __device__(const TVoxel1& voxel1) { return true; };
		auto return_true2 = [] __device__(const TVoxel2& voxel2) { return true; };

		return TraverseAndCompareAll_Generic<true>(
				volume1, volume2, functor,
				return_true1,
				return_true2
		);
	}

	template<typename TFunctor>
	inline static void
	TraverseAll(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	            VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	            TFunctor& functor) {
		TraverseAll_Generic(volume1, volume2, functor, [](
				dim3 hash_per_block_grid_size, dim3 block_voxel_per_thread_block_size,
				TVoxel1* voxels1, TVoxel2* voxels2, const HashEntry* hash_table1,
				const HashEntry* hash_table2, TFunctor* functor_device) {
			traverseAll_device<TFunctor, TVoxel1, TVoxel2>
			<<< hash_per_block_grid_size, block_voxel_per_thread_block_size >>>
					(voxels1, voxels2, hash_table1, hash_table2, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseUtilized(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                 VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                 TFunctor& functor) {
		TraverseUtilized_Generic(volume1, volume2, functor, [](
				dim3 hash_per_block_grid_size, dim3 block_voxel_per_thread_block_size,
				TVoxel1* voxels1, TVoxel2* voxels2, const HashEntry* hash_table1,
				const HashEntry* hash_table2, const int* utilized_hash_codes, TFunctor* functor_device) {
			traverseUtilized_device<TFunctor, TVoxel1, TVoxel2>
			<<< hash_per_block_grid_size, block_voxel_per_thread_block_size >>>
					(voxels1, voxels2, hash_table1, hash_table2, utilized_hash_codes, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	TraverseAllWithPosition(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                        VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                        TFunctor& functor) {
		TraverseAll_Generic(volume1, volume2, functor, [](
				dim3 hash_per_block_grid_size, dim3 block_voxel_per_thread_block_size,
				TVoxel1* voxels1, TVoxel2* voxels2, const HashEntry* hash_table1,
				const HashEntry* hash_table2, TFunctor* functor_device) {
			traverseAllWithPosition_device<TFunctor, TVoxel1, TVoxel2>
			<<< hash_per_block_grid_size, block_voxel_per_thread_block_size >>>
					(voxels1, voxels2, hash_table1, hash_table2, functor_device);
		});
	}

	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TFunctor>
	inline static void
	TraverseUtilizedWithPosition(VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
	                             VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
	                             TFunctor& functor) {
		TraverseUtilized_Generic(volume1, volume2, functor, [](
				dim3 hash_per_block_grid_size, dim3 block_voxel_per_thread_block_size,
				TVoxel1* voxels1, TVoxel2* voxels2, const HashEntry* hash_table1,
				const HashEntry* hash_table2, const int* utilized_hash_codes,
				TFunctor* functor_device) {
			traverseUtilizedWithPosition_device<TFunctor, TVoxel1, TVoxel2>
			<<< hash_per_block_grid_size, block_voxel_per_thread_block_size >>>
					(voxels1, voxels2, hash_table1, hash_table2, utilized_hash_codes, functor_device);
		});
	}


	template<ExecutionMode TExecutionMode = ExecutionMode::OPTIMIZED, typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareMatchingFlags(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelFlags flags,
			TBooleanFunctor& functor,
			bool verbose) {
		auto flags_match1 = [=] __device__(const TVoxel1& voxel1) { return voxel1.flags == flags; };
		auto flags_match2 = [=] __device__(const TVoxel2& voxel2) { return voxel2.flags == flags; };
		//TODO: implement verbose functionality
		if (verbose == true) {
			DIEWITHEXCEPTION_REPORTLOCATION("Verbose functionality not implemented!");
		}
		return TraverseAndCompareAll_Generic<false>(
				volume1, volume2, functor,
				flags_match1,
				flags_match2
		);
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareMatchingFlagsWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			VoxelFlags flags,
			TBooleanFunctor& functor,
			bool verbose) {
		auto flags_match1 = [=] __device__(const TVoxel1& voxel1) { return voxel1.flags == flags; };
		auto flags_match2 = [=] __device__(const TVoxel2& voxel2) { return voxel2.flags == flags; };
		//TODO: implement verbose functionality
		if (verbose == true) {
			DIEWITHEXCEPTION_REPORTLOCATION("Verbose functionality not implemented!");
		}
		return TraverseAndCompareAll_Generic<true>(
				volume1, volume2, functor,
				flags_match1,
				flags_match2
		);
	}

};

} // namespace ITMLib