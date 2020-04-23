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
#include "TwoVolumeTraversal_CUDA_VoxelBlockHash_Kernels.h"

namespace ITMLib {


template<typename TVoxel1, typename TVoxel2>
class TwoVolumeTraversalEngine<TVoxel1, TVoxel2, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CUDA> {
private:

	template<typename TFunctor, typename TDeviceTraversalFunction>
	inline static void
	TraverseAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {
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

		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(
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

	template<typename TBooleanFunctor, typename TDeviceTraversalFunction>
	inline static bool
	TraverseAndCompareAll_Generic(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {
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
		ORUtils::MemoryBlock<UnmatchedHash> unmatchedHashes(hash_entry_count, true, true);

		// these will be needed for various matching & traversal operations
		TVoxel1* voxels1 = volume1->GetVoxels();
		TVoxel2* voxels2 = volume2->GetVoxels();
		const HashEntry* hash_table1 = volume1->index.GetIndexData();
		const HashEntry* hash_table2 = volume2->index.GetIndexData();

		dim3 cudaBlockSize_HashPerThread(256, 1);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hash_entry_count) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));
		matchUpHashEntriesByPosition
				<<< gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >>>
		                                          (hash_table1, hash_table2, hash_entry_count,
				                                          matched_hash_code_pairs.GetData(MEMORYDEVICE_CUDA),
				                                          unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
				                                          hash_match_info.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// check unmatched hashes
		hash_match_info.UpdateHostFromDevice();
		dim3 block_voxel_per_thread_block_size(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		int unmatchedHashCount = hash_match_info.GetData(MEMORYDEVICE_CPU)->unmatched_hash_count;
		if (unmatchedHashCount > 0) {
			dim3 gridSize_UnmatchedBlocks(unmatchedHashCount);
			checkIfUnmatchedVoxelBlocksAreAltered
					<<< gridSize_UnmatchedBlocks, block_voxel_per_thread_block_size >>>
			                                       (voxels1, voxels2,
					                                       hash_table1, hash_table2,
					                                       unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
					                                       hash_match_info.GetData(MEMORYDEVICE_CUDA),
					                                       mismatch_encountered.GetData(MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}

		// if an unmatched block in either volume was altered, return false
		mismatch_encountered.UpdateHostFromDevice();
		if (*mismatch_encountered.GetData(MEMORYDEVICE_CPU)) return false;

		int matchedHashCount = hash_match_info.GetData(MEMORYDEVICE_CPU)->matched_hash_count;

		if (matchedHashCount == 0) return true;

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		// perform voxel traversal on the CUDA, matching individual voxels at corresponding locations
		dim3 gridSize_MatchedBlocks(matchedHashCount);

		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(
				gridSize_MatchedBlocks, block_voxel_per_thread_block_size,
				voxels1, voxels2, hash_table1, hash_table2,
				matched_hash_code_pairs.GetData(MEMORYDEVICE_CUDA),
				hash_match_info.GetData(MEMORYDEVICE_CUDA),
				functor_device, mismatch_encountered.GetData(MEMORYDEVICE_CUDA));

		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

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

		return TraverseAndCompareAll_Generic(
				volume1, volume2, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxel1* voxels1, TVoxel2* voxels2,
				    const HashEntry* hash_table1_device, const HashEntry* hash_table2_device,
				    const HashPair* matchedHashes_device, const HashMatchInfo* matchInfo_device,
				    TBooleanFunctor* functor_device,
				    bool* falseEncountered) {
					checkIfMatchingHashBlockVoxelsYieldTrue<TBooleanFunctor, TVoxel1, TVoxel2>
							<<< gridSize, cudaBlockSize >>>
					                       (voxels1, voxels2, hash_table1_device, hash_table2_device,
							                       matchedHashes_device, matchInfo_device, functor_device, falseEncountered);

				}
		);
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxel1, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxel2, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {

		return TraverseAndCompareAll_Generic(
				volume1, volume2, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxel1* voxels1, TVoxel2* voxels2,
				    const HashEntry* hash_table1, const HashEntry* hash_table2,
				    const HashPair* matched_hash_codes, const HashMatchInfo* match_info,
				    TBooleanFunctor* functor_device,
				    bool* falseEncountered) {
					checkIfMatchingHashBlockVoxelsYieldTrue_Position<TBooleanFunctor, TVoxel1, TVoxel2>
							<<< gridSize, cudaBlockSize >>>
					                       (voxels1, voxels2, hash_table1, hash_table2,
							                       matched_hash_codes, match_info, functor_device, falseEncountered);

				}
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

	template<typename TFunctor>
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

};

} // namespace ITMLib