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
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"

namespace {
//CUDA kernels

__global__ void findBlocksNotSpannedByArray(
		const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
		const ITMHashEntry* hashTable, int hashEntryCount,
		int* hashCodesNotSpanned, int* countBlocksNotSpanned) {
	int hashCode = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	if (hashCode > hashEntryCount) return;
	const ITMHashEntry& hashEntry = hashTable[hashCode];
	if (hashEntry.ptr < 0)
		return;
	Vector3i blockMinVoxels = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
	Vector3i blockMaxVoxels = blockMinVoxels + Vector3i(VOXEL_BLOCK_SIZE);
	Vector3i arrayMinVoxels = arrayInfo->offset;
	Vector3i arrayMaxVoxels = arrayInfo->offset + arrayInfo->size;

	if (blockMaxVoxels.x < arrayMinVoxels.x || blockMinVoxels.x > arrayMaxVoxels.x ||
	    blockMaxVoxels.y < arrayMinVoxels.y || blockMinVoxels.y > arrayMaxVoxels.y ||
	    blockMaxVoxels.z < arrayMinVoxels.z || blockMinVoxels.z > arrayMaxVoxels.z) {
		int unspannedHashCodeIndex = atomicAdd(countBlocksNotSpanned, 1);
		hashCodesNotSpanned[unspannedHashCodeIndex] = hashCode;
	}
}


template<typename TVoxel>
__global__ void
checkIfHashVoxelBlocksHaveAlteredVoxelsWithSpecificFlags(const TVoxel* voxels, const ITMHashEntry* hashTable,
                                                         int* hashesToCheck, int countHashesToCheck,
                                                         bool* alteredBlockEncountered,
                                                         ITMLib::VoxelFlags flags) {
	if (*alteredBlockEncountered) return;
	int hashToCheckIdx = static_cast<int>(blockIdx.x);
	if (hashToCheckIdx > countHashesToCheck) return;
	int hash = hashesToCheck[hashToCheckIdx];
	const ITMHashEntry& hashEntry = hashTable[hash];
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	const TVoxel& voxel = voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId];
	if (isAltered(voxel) && voxel.flags == flags) {
		*alteredBlockEncountered = true;
	}
}

template<typename TVoxel>
__global__ void
checkIfHashVoxelBlocksAreAltered(const TVoxel* voxels, const ITMHashEntry* hashTable, int* hashesToCheck,
                                 int countHashesToCheck, bool* alteredBlockEncountered) {
	if (*alteredBlockEncountered) return;
	int hashToCheckIdx = static_cast<int>(blockIdx.x);
	if (hashToCheckIdx > countHashesToCheck) return;
	int hash = hashesToCheck[hashToCheckIdx];
	const ITMHashEntry& hashEntry = hashTable[hash];
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	if (isAltered(voxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + locId])) {
		*alteredBlockEncountered = true;
	}
}


template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfArrayContentIsUnalteredOrYieldsTrue(
		TVoxelArray* arrayVoxels,
		const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
		TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		const Vector3i minArrayCoord,
		const Vector3i maxArrayCoord, const Vector3s minBlockPos,
		TBooleanFunctor* functor, bool* falseOrAlteredEncountered) {

	if (*falseOrAlteredEncountered) return;

	int xVoxel = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	int yVoxel = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
	int zVoxel = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);

	Vector3i indexingCoord(xVoxel, yVoxel, zVoxel);
	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3s blockPosition(blockIdx.x, blockIdx.y, blockIdx.z);
	blockPosition += minBlockPos;

	int hash;
	if (xVoxel < minArrayCoord.x || xVoxel >= maxArrayCoord.x ||
	    yVoxel < minArrayCoord.y || yVoxel >= maxArrayCoord.y ||
	    zVoxel < minArrayCoord.z || zVoxel >= maxArrayCoord.z) {
		if (FindHashAtPosition(hash, blockPosition, hashTable) &&
		    isAltered(hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock])) {
			// voxel falls just outside of array BUT is still in hash block, if it's altered -- return false
			*falseOrAlteredEncountered = true;
			return;
		}

		return;
	}

	Vector3i arrayCoord = indexingCoord - minArrayCoord;
	int idxInArray = arrayCoord.x + arrayCoord.y * arrayInfo->size.x +
	                 arrayCoord.z * arrayInfo->size.x * arrayInfo->size.y;

	if (FindHashAtPosition(hash, blockPosition, hashTable)) {
		if (!(*functor)(arrayVoxels[idxInArray], hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock])) {
			*falseOrAlteredEncountered = true;
		}
	} else {
		if (isAltered(arrayVoxels[idxInArray])) {
			*falseOrAlteredEncountered = true;
		}
	}
}


template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfArrayContentIsUnalteredOrYieldsTrue_Position_Verbose(
		TVoxelArray* arrayVoxels,
		const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
		TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		const Vector3i minArrayCoord,
		const Vector3i maxArrayCoord, const Vector3s minBlockPos,
		TBooleanFunctor* functor, bool* falseOrAlteredEncountered) {

	if (*falseOrAlteredEncountered) return;

	int xVoxel = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	int yVoxel = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
	int zVoxel = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);

	Vector3i indexingCoord(xVoxel, yVoxel, zVoxel);
	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3s blockPosition(blockIdx.x, blockIdx.y, blockIdx.z);
	blockPosition += minBlockPos;
	Vector3i voxelPosition = blockPosition.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);

	int hash;
	if (xVoxel < minArrayCoord.x || xVoxel >= maxArrayCoord.x ||
	    yVoxel < minArrayCoord.y || yVoxel >= maxArrayCoord.y ||
	    zVoxel < minArrayCoord.z || zVoxel >= maxArrayCoord.z) {
		if (FindHashAtPosition(hash, blockPosition, hashTable) &&
		    isAltered_VerbosePositionHash(hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock],
		                                  voxelPosition, hash, blockPosition, "In voxel-block-hash volume: ")) {
			// voxel falls just outside of array BUT is still in hash block, if it's altered -- return false
			*falseOrAlteredEncountered = true;
			return;
		}

		return;
	}

	Vector3i arrayCoord = indexingCoord - minArrayCoord;
	int idxInArray = arrayCoord.x + arrayCoord.y * arrayInfo->size.x +
	                 arrayCoord.z * arrayInfo->size.x * arrayInfo->size.y;

	if (FindHashAtPosition(hash, blockPosition, hashTable)) {

		if (!(*functor)(arrayVoxels[idxInArray], hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock],
		                voxelPosition)) {
			*falseOrAlteredEncountered = true;
		}
	} else {
		if (isAltered_VerbosePosition(arrayVoxels[idxInArray], voxelPosition, "In plain-voxel-array volume: ")) {
			*falseOrAlteredEncountered = true;
		}
	}
}


template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfArrayContentIsUnalteredOrYieldsTrue_SemanticFlags(
		TVoxelArray* arrayVoxels,
		const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
		TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		const Vector3i minArrayCoord,
		const Vector3i maxArrayCoord, const Vector3s minBlockPos,
		TBooleanFunctor* functor, ITMLib::VoxelFlags semanticFlags,
		bool* falseOrAlteredUnmatchedEncountered) {

	if (*falseOrAlteredUnmatchedEncountered) return;

	int xVoxel = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	int yVoxel = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
	int zVoxel = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);

	Vector3i indexingCoord(xVoxel, yVoxel, zVoxel);
	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3s blockPosition(blockIdx.x, blockIdx.y, blockIdx.z);
	blockPosition += minBlockPos;

	int hash;
	if (xVoxel < minArrayCoord.x || xVoxel >= maxArrayCoord.x ||
	    yVoxel < minArrayCoord.y || yVoxel >= maxArrayCoord.y ||
	    zVoxel < minArrayCoord.z || zVoxel >= maxArrayCoord.z) {
		if (FindHashAtPosition(hash, blockPosition, hashTable)) {
			TVoxelHash& voxel = hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];
			if (voxel.flags == semanticFlags && isAltered(voxel)) {
				// voxel falls just outside of array BUT is still in hash block, if it's altered & has same flags -- return false
				*falseOrAlteredUnmatchedEncountered = true;
				return;
			}
		}

		return;
	}

	Vector3i arrayCoord = indexingCoord - minArrayCoord;
	int idxInArray = arrayCoord.x + arrayCoord.y * arrayInfo->size.x +
	                 arrayCoord.z * arrayInfo->size.x * arrayInfo->size.y;

	if (FindHashAtPosition(hash, blockPosition, hashTable)) {
		TVoxelHash& voxelHash = hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];
		TVoxelArray& voxelArray = arrayVoxels[idxInArray];
		if ((voxelHash.flags == semanticFlags || voxelArray.flags == semanticFlags) &&
		    (!(*functor)(voxelArray, voxelHash) || voxelHash.flags != voxelArray.flags)) {
			*falseOrAlteredUnmatchedEncountered = true;
		}
	} else {
		TVoxelArray& voxelArray = arrayVoxels[idxInArray];
		if (voxelArray.flags == semanticFlags && isAltered(voxelArray)) {
			*falseOrAlteredUnmatchedEncountered = true;
		}
	}
}


template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfArrayContentIsUnalteredOrYieldsTrue_SemanticFlags_Position_Verbose(
		TVoxelArray* arrayVoxels,
		const ITMLib::PlainVoxelArray::GridAlignedBox* arrayInfo,
		TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		const Vector3i minArrayCoord,
		const Vector3i maxArrayCoord, const Vector3s minBlockPos,
		TBooleanFunctor* functor, ITMLib::VoxelFlags semanticFlags,
		bool* falseOrAlteredUnmatchedEncountered) {

	if (*falseOrAlteredUnmatchedEncountered) return;

	int xVoxel = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	int yVoxel = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
	int zVoxel = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);

	Vector3i indexingCoord(xVoxel, yVoxel, zVoxel);
	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3s blockPosition(blockIdx.x, blockIdx.y, blockIdx.z);
	blockPosition += minBlockPos;
	Vector3i voxelPosition = blockPosition.toInt() * VOXEL_BLOCK_SIZE + Vector3i(x, y, z);

	int hash;
	if (xVoxel < minArrayCoord.x || xVoxel >= maxArrayCoord.x ||
	    yVoxel < minArrayCoord.y || yVoxel >= maxArrayCoord.y ||
	    zVoxel < minArrayCoord.z || zVoxel >= maxArrayCoord.z) {
		if (FindHashAtPosition(hash, blockPosition, hashTable)) {
			TVoxelHash& voxel = hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];

			if (voxel.flags == semanticFlags &&
			    isAltered_VerbosePositionHash(voxel, voxelPosition, hash, blockPosition,
			                                  "In voxel-block-hash volume: ")) {
				// voxel falls just outside of array BUT is still in hash block, if it's altered & has same flags -- return false
				*falseOrAlteredUnmatchedEncountered = true;
				return;
			}
		}

		return;
	}

	Vector3i arrayCoord = indexingCoord - minArrayCoord;
	int idxInArray = arrayCoord.x + arrayCoord.y * arrayInfo->size.x +
	                 arrayCoord.z * arrayInfo->size.x * arrayInfo->size.y;

	if (FindHashAtPosition(hash, blockPosition, hashTable)) {
		TVoxelHash& voxelHash = hashVoxels[hashTable[hash].ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];
		TVoxelArray& voxelArray = arrayVoxels[idxInArray];
		if ((voxelHash.flags == semanticFlags || voxelArray.flags == semanticFlags) &&
		    (!(*functor)(voxelArray, voxelHash, voxelPosition) || voxelHash.flags != voxelArray.flags)) {
			*falseOrAlteredUnmatchedEncountered = true;
		}
	} else {
		TVoxelArray& voxelArray = arrayVoxels[idxInArray];
		if (voxelArray.flags == semanticFlags && isAltered_VerbosePosition(voxelArray, voxelPosition, "In plain-voxel-array volume: ")) {
			*falseOrAlteredUnmatchedEncountered = true;
		}
	}
}

template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfAllocatedHashBlocksYieldTrue(
		TVoxelArray* arrayVoxels, TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		int hashEntryCount, Vector6i arrayBounds, Vector3i arraySize,
		TBooleanFunctor* functor, bool* falseEncountered) {

	if (*falseEncountered || blockIdx.x > hashEntryCount || hashTable[blockIdx.x].ptr < 0) return;

	const ITMHashEntry& hashEntry = hashTable[blockIdx.x];

	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3i globalPosition = (hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE) + Vector3i(x, y, z);

	if (globalPosition.x < arrayBounds.min_x || globalPosition.x >= arrayBounds.max_x ||
	    globalPosition.y < arrayBounds.min_y || globalPosition.y >= arrayBounds.max_y ||
	    globalPosition.z < arrayBounds.min_z || globalPosition.z >= arrayBounds.max_z) {
		// outside of the array bounds
		return;
	}

	Vector3i voxelPositionSansOffset =
			globalPosition - Vector3i(arrayBounds.min_x, arrayBounds.min_y, arrayBounds.min_z);
	int idxInArray = voxelPositionSansOffset.x + voxelPositionSansOffset.y * arraySize.x +
	                 voxelPositionSansOffset.z * arraySize.x * arraySize.y;

	TVoxelHash hashVoxel = hashVoxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];
	TVoxelArray arrayVoxel = arrayVoxels[idxInArray];

	if (!(*functor)(arrayVoxel, hashVoxel)) {
		*falseEncountered = true;
	}
}


template<typename TBooleanFunctor, typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfAllocatedHashBlocksYieldTrue_Position(
		TVoxelArray* arrayVoxels, TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		int hashEntryCount, Vector6i arrayBounds, Vector3i arraySize,
		TBooleanFunctor* functor, bool* falseEncountered) {

	if (*falseEncountered || blockIdx.x > hashEntryCount || hashTable[blockIdx.x].ptr < 0) return;

	const ITMHashEntry& hashEntry = hashTable[blockIdx.x];

	//local (block) coords;
	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;
	int idxInBlock = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

	Vector3i globalPosition = (hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE) + Vector3i(x, y, z);

	if (globalPosition.x < arrayBounds.min_x || globalPosition.x >= arrayBounds.max_x ||
	    globalPosition.y < arrayBounds.min_y || globalPosition.y >= arrayBounds.max_y ||
	    globalPosition.z < arrayBounds.min_z || globalPosition.z >= arrayBounds.max_z) {
		// outside of the array bounds
		return;
	}

	Vector3i voxelPositionSansOffset =
			globalPosition - Vector3i(arrayBounds.min_x, arrayBounds.min_y, arrayBounds.min_z);
	int idxInArray = voxelPositionSansOffset.x + voxelPositionSansOffset.y * arraySize.x +
	                 voxelPositionSansOffset.z * arraySize.x * arraySize.y;

	TVoxelHash hashVoxel = hashVoxels[hashEntry.ptr * VOXEL_BLOCK_SIZE3 + idxInBlock];
	TVoxelArray arrayVoxel = arrayVoxels[idxInArray];

	if (!(*functor)(arrayVoxel, hashVoxel, globalPosition)) {
		*falseEncountered = true;
	}
}


} // anonymous namespace (CUDA kernels)