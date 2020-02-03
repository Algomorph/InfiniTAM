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

namespace ITMLib{


template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, VoxelBlockHash, VoxelBlockHash, MEMORYDEVICE_CUDA> {
private:

	template<typename TFunctor, typename TDeviceTraversalFunction>
	inline static void
	DualVoxelTraversal_Generic(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const HashEntry* primaryHashTable = primaryScene->index.GetIndexData();
		const HashEntry* secondaryHashTable = secondaryScene->index.GetIndexData();
		int hashEntryCount = primaryScene->index.hashEntryCount;

		// transfer functor from RAM to VRAM
		TFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hashEntryCount);

		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(
				gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread,
				primaryVoxels, secondaryVoxels, primaryHashTable, secondaryHashTable, functor_device);
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
	}

	template<typename TBooleanFunctor, typename TDeviceTraversalFunction>
	inline static bool
	DualVoxelTraversal_AllTrue_Generic(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TBooleanFunctor& functor, TDeviceTraversalFunction&& deviceTraversalFunction) {
		// assumes functor is allocated in main memory

		int hashEntryCount = primaryScene->index.hashEntryCount;

		// allocate intermediate-result buffers for use on the CUDA in subsequent routine calls
		ORUtils::MemoryBlock<bool> badResultEncountered_device(1, true, true);
		*badResultEncountered_device.GetData(MEMORYDEVICE_CPU) = false;
		badResultEncountered_device.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashMatchInfo> hashMatchInfo(1, true, true);
		hashMatchInfo.GetData(MEMORYDEVICE_CPU)->matchedHashCount = 0;
		hashMatchInfo.GetData(MEMORYDEVICE_CPU)->unmatchedHashCount = 0;
		hashMatchInfo.UpdateDeviceFromHost();
		ORUtils::MemoryBlock<HashPair> matchedHashPairs(hashEntryCount, true, true);
		ORUtils::MemoryBlock<UnmatchedHash> unmatchedHashes(hashEntryCount, true, true);

		// these will be needed for various matching & traversal operations
		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const HashEntry* primaryHashTable = primaryScene->index.GetIndexData();
		const HashEntry* secondaryHashTable = secondaryScene->index.GetIndexData();

		dim3 cudaBlockSize_HashPerThread(256, 1);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hashEntryCount) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));
		matchUpHashEntriesByPosition
				<< < gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >> >
		                                          (primaryHashTable, secondaryHashTable, hashEntryCount,
				                                          matchedHashPairs.GetData(MEMORYDEVICE_CUDA),
				                                          unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
				                                          hashMatchInfo.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// check unmatched hashes
		hashMatchInfo.UpdateHostFromDevice();
		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		int unmatchedHashCount = hashMatchInfo.GetData(MEMORYDEVICE_CPU)->unmatchedHashCount;
		if (unmatchedHashCount > 0) {
			dim3 gridSize_UnmatchedBlocks(unmatchedHashCount);
			checkIfUnmatchedVoxelBlocksAreAltered
					<< < gridSize_UnmatchedBlocks, cudaBlockSize_BlockVoxelPerThread >> >
			                                       (primaryVoxels, secondaryVoxels,
					                                       primaryHashTable, secondaryHashTable,
					                                       unmatchedHashes.GetData(MEMORYDEVICE_CUDA),
					                                       hashMatchInfo.GetData(MEMORYDEVICE_CUDA),
					                                       badResultEncountered_device.GetData(MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}

		// if an unmatched block in either volume was altered, return false
		badResultEncountered_device.UpdateHostFromDevice();
		if (*badResultEncountered_device.GetData(MEMORYDEVICE_CPU)) return false;

		int matchedHashCount = hashMatchInfo.GetData(MEMORYDEVICE_CPU)->matchedHashCount;

		if (matchedHashCount == 0) return true;

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		// perform voxel traversal on the CUDA, matching individual voxels at corresponding locations
		dim3 gridSize_MatchedBlocks(matchedHashCount);

		std::forward<TDeviceTraversalFunction>(deviceTraversalFunction)(
				gridSize_MatchedBlocks, cudaBlockSize_BlockVoxelPerThread,
				primaryVoxels, secondaryVoxels, primaryHashTable, secondaryHashTable,
				matchedHashPairs.GetData(MEMORYDEVICE_CUDA),
				hashMatchInfo.GetData(MEMORYDEVICE_CUDA),
				functor_device,	badResultEncountered_device.GetData(MEMORYDEVICE_CUDA));

		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));

		badResultEncountered_device.UpdateHostFromDevice();
		return !(*badResultEncountered_device.GetData(MEMORYDEVICE_CPU));
	}

public:
	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TBooleanFunctor& functor) {

		return DualVoxelTraversal_AllTrue_Generic(
				primaryScene, secondaryScene, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
				    const HashEntry* primaryHashTable_device, const HashEntry* secondaryHashTable_device,
				    const HashPair* matchedHashes_device, const HashMatchInfo* matchInfo_device, TBooleanFunctor* functor_device,
				    bool* falseEncountered) {
					checkIfMatchingHashBlockVoxelsYieldTrue<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
					<< < gridSize, cudaBlockSize >> >
					(primaryVoxels, secondaryVoxels, primaryHashTable_device, secondaryHashTable_device,
							matchedHashes_device,matchInfo_device,functor_device,falseEncountered);

				}
		);
	}

	template<typename TBooleanFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TBooleanFunctor& functor) {

		return DualVoxelTraversal_AllTrue_Generic(
				primaryScene, secondaryScene, functor,
				[&](const dim3& gridSize, const dim3& cudaBlockSize,
				    TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
				    const HashEntry* primaryHashTable_device, const HashEntry* secondaryHashTable_device,
				    const HashPair* matchedHashes_device, const HashMatchInfo* matchInfo_device, TBooleanFunctor* functor_device,
				    bool* falseEncountered) {
					checkIfMatchingHashBlockVoxelsYieldTrue_Position<TBooleanFunctor, TVoxelPrimary, TVoxelSecondary>
					<< < gridSize, cudaBlockSize >> >
					(primaryVoxels, secondaryVoxels, primaryHashTable_device, secondaryHashTable_device,
							matchedHashes_device,matchInfo_device,functor_device,falseEncountered);

				}
		);
	}

	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {
		DualVoxelTraversal_Generic(primaryScene,secondaryScene,functor, [](
				dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
				TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels, const HashEntry* primaryHashTable,
				const HashEntry* secondaryHashTable, TFunctor* functor_device){
			dualVoxelTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
			<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
			(primaryVoxels, secondaryVoxels, primaryHashTable,
					secondaryHashTable, functor_device);
		});
	}

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* primaryScene,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* secondaryScene,
			TFunctor& functor) {
		DualVoxelTraversal_Generic(primaryScene,secondaryScene,functor, [](
				dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread,
				TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels, const HashEntry* primaryHashTable,
				const HashEntry* secondaryHashTable, TFunctor* functor_device){
			dualVoxelPositionTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
			<< < gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >> >
			(primaryVoxels, secondaryVoxels, primaryHashTable,
					secondaryHashTable, functor_device);
		});
	}

};

} // namespace ITMLib