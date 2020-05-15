//  ================================================================
//  Created by Gregory Kramida on 10/4/19.
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


//local
#include "../Shared/VolumeTraversal_Shared.h"
#include "../Interface/TwoVolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/Analytics/IsAltered.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "TwoVolumeTraversal_CUDA_PVA_to_VBH_Kernels.h"

namespace ITMLib {


template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CUDA> {
	//TODO: combine TraverseAndCompareMatchingFlags_Generic with TraversalAndCompareAll_Generic the same
	// way as done in the the CPU version to avoid DRY violations
	template<typename TBooleanFunctor, typename TDeviceFunction>
	inline static bool
	TraverseAndCompareMatchingFlags_Generic(VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
	                                        VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
	                                        VoxelFlags semanticFlags, TBooleanFunctor& functor,
	                                        TDeviceFunction&& deviceFunction) {
		GridAlignedBox* array_index_data = volume1->index.GetIndexData();
		int hash_entry_count = volume2->index.hash_entry_count;
		HashEntry* hash_table = volume2->index.GetIndexData();

		ORUtils::MemoryBlock<int> hash_blocks_outside_array(hash_entry_count, true, true);
		ORUtils::MemoryBlock<int> count_hash_blocks_outside_array(1, true, true);
		*count_hash_blocks_outside_array.GetData(MEMORYDEVICE_CPU) = 0;
		count_hash_blocks_outside_array.UpdateDeviceFromHost();

		dim3 cudaBlockSize_HashPerThread(256);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hash_entry_count) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));
		findBlocksNotSpannedByArray <<< gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >>>
		                                                              (array_index_data, hash_table, hash_entry_count,
				                                                              hash_blocks_outside_array.GetData(
						                                                              MEMORYDEVICE_CUDA),
				                                                              count_hash_blocks_outside_array.GetData(
						                                                              MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;
		count_hash_blocks_outside_array.UpdateHostFromDevice();
		int count_hash_blocks_outside_array_host = *count_hash_blocks_outside_array.GetData(MEMORYDEVICE_CPU);

		//*** used in the next two kernels
		ORUtils::MemoryBlock<bool> falseOrAlteredUnmatchedEncountered(1, true, true);
		*falseOrAlteredUnmatchedEncountered.GetData(MEMORYDEVICE_CPU) = false;
		falseOrAlteredUnmatchedEncountered.UpdateDeviceFromHost();
		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		TVoxelSecondary* voxels_hash = volume2->GetVoxels();

		if (count_hash_blocks_outside_array_host > 0) {
			// if there are any blocks in the hash that are not spanned by the array volume, we must check
			// whether any of them are altered
			dim3 gridSize_UnspannedHashBlocks(static_cast<int>(ceil(static_cast<float>(count_hash_blocks_outside_array_host) /
			                                                        static_cast<float>(cudaBlockSize_HashPerThread.x))));
			checkIfHashVoxelBlocksHaveAlteredVoxelsWithSpecificFlags
					<<< gridSize_UnspannedHashBlocks, cudaBlockSize_BlockVoxelPerThread >>>
			                                           (voxels_hash, hash_table, hash_blocks_outside_array.GetData(
					                                           MEMORYDEVICE_CUDA),
					                                           count_hash_blocks_outside_array_host,
					                                           falseOrAlteredUnmatchedEncountered.GetData(
							                                           MEMORYDEVICE_CUDA), semanticFlags);
			ORcudaKernelCheck;
		}
		falseOrAlteredUnmatchedEncountered.UpdateHostFromDevice();
		if (*falseOrAlteredUnmatchedEncountered.GetData(MEMORYDEVICE_CPU)) {
			return false;
		}

		TVoxelPrimary* voxels_array = volume1->GetVoxels();
		Vector3i array_offset = volume1->index.GetVolumeOffset();
		Vector3i array_size = volume1->index.GetVolumeSize();

		Vector3s minBlockPos(
				static_cast<int>(floor(static_cast<float>(array_offset.x) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(array_offset.y) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(array_offset.z) / VOXEL_BLOCK_SIZE)));
		Vector3i minIndexCoord(minBlockPos.x * VOXEL_BLOCK_SIZE,
		                       minBlockPos.y * VOXEL_BLOCK_SIZE,
		                       minBlockPos.z * VOXEL_BLOCK_SIZE);
		Vector3i minArrayCoord = array_offset - minIndexCoord; // inclusive
		Vector3i maxArrayCoord = minArrayCoord + array_size; // exclusive

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		dim3 gridSize_ArrayBlockEnvelope(
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.x) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.y) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.z) / VOXEL_BLOCK_SIZE))
		);

		std::forward<TDeviceFunction>(deviceFunction)(
				gridSize_ArrayBlockEnvelope, cudaBlockSize_BlockVoxelPerThread, voxels_array, array_index_data, voxels_hash,
				hash_table, minArrayCoord, maxArrayCoord, minBlockPos, functor_device, semanticFlags,
				falseOrAlteredUnmatchedEncountered.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		falseOrAlteredUnmatchedEncountered.UpdateHostFromDevice();

		return !(*falseOrAlteredUnmatchedEncountered.GetData(MEMORYDEVICE_CPU));
	}


	template<typename TBooleanFunctor, typename TDeviceFunction>
	inline static bool
	TraverseAndCompareAll_Generic(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor, TDeviceFunction&& deviceFunction) {
		GridAlignedBox* array_index_data = volume1->index.GetIndexData();
		int hash_entry_count = volume2->index.hash_entry_count;
		HashEntry* hash_table = volume2->index.GetIndexData();

		ORUtils::MemoryBlock<int> hashCodesNotSpanned(hash_entry_count, true, true);
		ORUtils::MemoryBlock<int> countBlocksNotSpanned(1, true, true);
		*countBlocksNotSpanned.GetData(MEMORYDEVICE_CPU) = 0;
		countBlocksNotSpanned.UpdateDeviceFromHost();

		dim3 cudaBlockSize_HashPerThread(256);
		dim3 gridSize_MultipleHashBlocks(static_cast<int>(ceil(static_cast<float>(hash_entry_count) /
		                                                       static_cast<float>(cudaBlockSize_HashPerThread.x))));
		findBlocksNotSpannedByArray
				<<< gridSize_MultipleHashBlocks, cudaBlockSize_HashPerThread >>>
		                                          (array_index_data, hash_table, hash_entry_count,
		                                          		hashCodesNotSpanned.GetData(MEMORYDEVICE_CUDA),
				                                          countBlocksNotSpanned.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;
		countBlocksNotSpanned.UpdateHostFromDevice();
		int count_hash_blocks_outside_array_host = *countBlocksNotSpanned.GetData(MEMORYDEVICE_CPU);

		//*** used in the next two kernels
		ORUtils::MemoryBlock<bool> falseOrAlteredEncountered(1, true, true);
		*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU) = false;
		falseOrAlteredEncountered.UpdateDeviceFromHost();
		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		TVoxelSecondary* voxels_hash = volume2->GetVoxels();

		if (count_hash_blocks_outside_array_host > 0) {
			// if there are any blocks in the hash that are not spanned by the array volume, we must check
			// whether any of them are altered
			dim3 gridSize_UnspannedHashBlocks(static_cast<int>(ceil(static_cast<float>(count_hash_blocks_outside_array_host) /
			                                                        static_cast<float>(cudaBlockSize_HashPerThread.x))));
			checkIfHashVoxelBlocksAreAltered
					<<< gridSize_UnspannedHashBlocks, cudaBlockSize_BlockVoxelPerThread >>>
			                                           (voxels_hash, hash_table, hashCodesNotSpanned.GetData(
					                                           MEMORYDEVICE_CUDA),
					                                           count_hash_blocks_outside_array_host,
					                                           falseOrAlteredEncountered.GetData(
							                                           MEMORYDEVICE_CUDA));
			ORcudaKernelCheck;
		}
		falseOrAlteredEncountered.UpdateHostFromDevice();
		if (*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU)) {
			return false;
		}

		TVoxelPrimary* voxels_array = volume1->GetVoxels();
		Vector3i array_offset = volume1->index.GetVolumeOffset();
		Vector3i array_size = volume1->index.GetVolumeSize();

		Vector3s minBlockPos(
				static_cast<int>(floor(static_cast<float>(array_offset.x) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(array_offset.y) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(floor(static_cast<float>(array_offset.z) / VOXEL_BLOCK_SIZE)));
		Vector3i minIndexCoord(minBlockPos.x * VOXEL_BLOCK_SIZE,
		                       minBlockPos.y * VOXEL_BLOCK_SIZE,
		                       minBlockPos.z * VOXEL_BLOCK_SIZE);
		Vector3i minArrayCoord = array_offset - minIndexCoord; // inclusive
		Vector3i maxArrayCoord = minArrayCoord + array_size; // exclusive

		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		dim3 gridSize_ArrayBlockEnvelope(
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.x) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.y) / VOXEL_BLOCK_SIZE)),
				static_cast<int>(ceil(static_cast<float>(maxArrayCoord.z) / VOXEL_BLOCK_SIZE))
		);

		std::forward<TDeviceFunction>(deviceFunction)(
				gridSize_ArrayBlockEnvelope, cudaBlockSize_BlockVoxelPerThread, voxels_array, array_index_data, voxels_hash,
				hash_table, minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
				falseOrAlteredEncountered.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;

		// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		falseOrAlteredEncountered.UpdateHostFromDevice();

		return !(*falseOrAlteredEncountered.GetData(MEMORYDEVICE_CPU));
	}

	template<typename TBooleanFunctor, typename TDeviceFunction>
	inline static bool
	TravereAndCompareAllocated_Generic(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor, TDeviceFunction&& deviceFunction) {

		int hash_entry_count = volume2->index.hash_entry_count;
		TVoxelSecondary* hashVoxels = volume2->GetVoxels();
		TVoxelPrimary* arrayVoxels = volume1->GetVoxels();
		const VoxelBlockHash::IndexData* hash_table = volume2->index.GetIndexData();
		const PlainVoxelArray::IndexData* array_index_data = volume1->index.GetIndexData();
		Vector3i startVoxel = volume1->index.GetVolumeOffset();
		Vector3i array_size = volume1->index.GetVolumeSize();
		Vector3i endVoxel = startVoxel + array_size; // open last traversal bound (this voxel doesn't get processed)
		Vector6i arrayBounds(startVoxel.x, startVoxel.y, startVoxel.z, endVoxel.x, endVoxel.y, endVoxel.z);

		// for result storage
		ORUtils::MemoryBlock<bool> falseEncountered(1, true, true);
		*falseEncountered.GetData(MEMORYDEVICE_CPU) = false;
		falseEncountered.UpdateDeviceFromHost();
		// transfer functor from RAM to VRAM (has to be done manually, i.e. without MemoryBlock at this point)
		TBooleanFunctor* functor_device = nullptr;
		ORcudaSafeCall(cudaMalloc((void**) &functor_device, sizeof(TBooleanFunctor)));
		ORcudaSafeCall(cudaMemcpy(functor_device, &functor, sizeof(TBooleanFunctor), cudaMemcpyHostToDevice));

		dim3 cudaBlockSize_BlockVoxelPerThread(VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE, VOXEL_BLOCK_SIZE);
		dim3 gridSize_HashPerBlock(hash_entry_count);

		std::forward<TDeviceFunction>(deviceFunction)(
				gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread, arrayVoxels, hashVoxels,
				hash_table, hash_entry_count, arrayBounds,
				array_size, functor_device, falseEncountered.GetData(MEMORYDEVICE_CUDA));
		ORcudaKernelCheck;
// transfer functor from VRAM back to RAM (in case there was any alteration to the functor object)
		ORcudaSafeCall(cudaMemcpy(&functor, functor_device, sizeof(TBooleanFunctor), cudaMemcpyDeviceToHost));
		ORcudaSafeCall(cudaFree(functor_device));
		falseEncountered.UpdateHostFromDevice();
		return !(*falseEncountered.GetData(MEMORYDEVICE_CPU));
	}

public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param volume1 the primary volume -- indexed using plain voxel array (PVA)
	 * \param volume2 the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {

		return TraverseAndCompareAll_Generic(volume1, volume2, functor, []
				(dim3 gridSize_ArrayBlockEnvelope, dim3 cudaBlockSize_BlockVoxelPerThread,
				 TVoxelPrimary* voxels_array, GridAlignedBox* array_index_data,
				 TVoxelSecondary* voxels_hash, HashEntry* hash_table, Vector3i minArrayCoord,
				 Vector3i maxArrayCoord, Vector3s minBlockPos, TBooleanFunctor* functor_device,
				 bool* falseOrAlteredEncountered_device) {
			checkIfArrayContentIsUnalteredOrYieldsTrue
					<<< gridSize_ArrayBlockEnvelope, cudaBlockSize_BlockVoxelPerThread >>>
			                                          (voxels_array, array_index_data, voxels_hash, hash_table,
					                                          minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
					                                          falseOrAlteredEncountered_device);
		});
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location and the same semantic flags.
	 *
	 * \details voxels that are not modified / have default value (see isModified for how that works) or don't have
	 * the specified semantic flags in the primary voxel volume are ignored if the voxel hash block at this location at
	 * the secondary voxel volume has not been allocated. Likewise, if hash blocks in the secondary volume fall outside
	 * the dense 3D array of the primary volume, their voxels are checked for alterations and flags. If they are not
	 * altered or their flags don't match the specified flags, they are ignored.
	 * \note works only for voxels that have semantic flags defined.
	 *
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param volume1 the primary volume -- indexed using plain voxel array (PVA)
	 * \param volume2 the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param semanticFlags semantic flags to match
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareMatchingFlags(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			VoxelFlags semanticFlags, TBooleanFunctor& functor) {

		return TraverseAndCompareMatchingFlags_Generic(volume1, volume2, semanticFlags, functor,
		                                               []
				                                               (dim3 gridSize_ArrayBlockEnvelope,
				                                                dim3 cudaBlockSize_BlockVoxelPerThread,
				                                                TVoxelPrimary* voxels_array,
				                                                GridAlignedBox* array_index_data,
				                                                TVoxelSecondary* voxels_hash,
				                                                HashEntry* hash_table,
				                                                Vector3i minArrayCoord,
				                                                Vector3i maxArrayCoord, Vector3s minBlockPos,
				                                                TBooleanFunctor* functor_device,
				                                                VoxelFlags semanticFlags,
				                                                bool* falseOrAlteredEncountered_device) {
			                                               checkIfArrayContentIsUnalteredOrYieldsTrue_SemanticFlags
					                                               <<< gridSize_ArrayBlockEnvelope,
					                                               cudaBlockSize_BlockVoxelPerThread >>>
					                                               (voxels_array, array_index_data, voxels_hash, hash_table,
							                                               minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
							                                               semanticFlags, falseOrAlteredEncountered_device);
		                                               });
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareMatchingFlagsWithPosition(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			VoxelFlags semanticFlags, TBooleanFunctor& functor) {

		return TraverseAndCompareMatchingFlags_Generic(volume1, volume2, semanticFlags, functor,
		                                               []
				                                               (dim3 gridSize_ArrayBlockEnvelope,
				                                                dim3 cudaBlockSize_BlockVoxelPerThread,
				                                                TVoxelPrimary* voxels_array,
				                                                GridAlignedBox* array_index_data,
				                                                TVoxelSecondary* voxels_hash,
				                                                HashEntry* hash_table,
				                                                Vector3i minArrayCoord,
				                                                Vector3i maxArrayCoord, Vector3s minBlockPos,
				                                                TBooleanFunctor* functor_device,
				                                                VoxelFlags semanticFlags,
				                                                bool* falseOrAlteredEncountered_device) {
			                                               checkIfArrayContentIsUnalteredOrYieldsTrue_SemanticFlags_Position_Verbose
					                                               <<< gridSize_ArrayBlockEnvelope,
					                                               cudaBlockSize_BlockVoxelPerThread >>>
					                                               (voxels_array, array_index_data, voxels_hash, hash_table,
							                                               minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
							                                               semanticFlags, falseOrAlteredEncountered_device);
		                                               });
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location (variant with functor accepting voxel position).
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param volume1 the primary volume -- indexed using plain voxel array (PVA)
	 * \param volume2 the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and their mutual spatial coordinate,
	 *  and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllWithPosition(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {

		return TraverseAndCompareAll_Generic(volume1, volume2, functor, []
				(dim3 gridSize_ArrayBlockEnvelope, dim3 cudaBlockSize_BlockVoxelPerThread,
				 TVoxelPrimary* voxels_array, GridAlignedBox* array_index_data,
				 TVoxelSecondary* voxels_hash, HashEntry* hash_table, Vector3i minArrayCoord,
				 Vector3i maxArrayCoord, Vector3s minBlockPos, TBooleanFunctor* functor_device,
				 bool* falseOrAlteredEncountered_device) {
			checkIfArrayContentIsUnalteredOrYieldsTrue_Position_Verbose
					<<< gridSize_ArrayBlockEnvelope, cudaBlockSize_BlockVoxelPerThread >>>
			                                          (voxels_array, array_index_data, voxels_hash, hash_table,
					                                          minArrayCoord, maxArrayCoord, minBlockPos, functor_device,
					                                          falseOrAlteredEncountered_device);
		});
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllocated(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {
		return TravereAndCompareAllocated_Generic(volume1, volume2, functor, []
				(dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread, TVoxelPrimary* arrayVoxels,
				 TVoxelSecondary* hashVoxels, const HashEntry* hash_table,
				 int hash_entry_count, Vector6i arrayBounds, Vector3i array_size,
				 TBooleanFunctor* functor_device, bool* falseEncountered_device) {
			checkIfAllocatedHashBlocksYieldTrue <<< gridSize_HashPerBlock, cudaBlockSize_BlockVoxelPerThread >>>
			                                                                (arrayVoxels, hashVoxels, hash_table, hash_entry_count, arrayBounds,
					                                                                array_size, functor_device, falseEncountered_device);
		});
	}

	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllocatedWithPosition(
			VoxelVolume<TVoxelPrimary, PlainVoxelArray>* volume1,
			VoxelVolume<TVoxelSecondary, VoxelBlockHash>* volume2,
			TBooleanFunctor& functor) {
		return TravereAndCompareAllocated_Generic(volume1, volume2, functor, []
				(dim3 gridSize_HashPerBlock, dim3 cudaBlockSize_BlockVoxelPerThread, TVoxelPrimary* arrayVoxels,
				 TVoxelSecondary* hashVoxels, const HashEntry* hash_table,
				 int hash_entry_count, Vector6i arrayBounds, Vector3i array_size,
				 TBooleanFunctor* functor_device, bool* falseEncountered_device) {
			checkIfAllocatedHashBlocksYieldTrue_Position <<< gridSize_HashPerBlock,
					cudaBlockSize_BlockVoxelPerThread >>>
					(arrayVoxels, hashVoxels, hash_table, hash_entry_count, arrayBounds,
							array_size, functor_device, falseEncountered_device);
		});
	}

};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, VoxelBlockHash, PlainVoxelArray, MEMORYDEVICE_CUDA> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param volume1 the primary volume -- indexed using plain voxel array (PVA)
	 * \param volume2 the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAll(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
			TBooleanFunctor& functor) {
		FlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TBooleanFunctor> flip_functor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CUDA>::
		TraverseAndCompareAll(volume2, volume1, flip_functor);

	}


	template<typename TBooleanFunctor>
	inline static bool
	TraverseAndCompareAllocated(
			VoxelVolume<TVoxelPrimary, VoxelBlockHash>* volume1,
			VoxelVolume<TVoxelSecondary, PlainVoxelArray>* volume2,
			TBooleanFunctor& functor) {
		FlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TBooleanFunctor> flip_functor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CUDA>::
		TraverseAndCompareAllocated(volume2, volume1, flip_functor);

	}
};
}//namespace ITMLib
