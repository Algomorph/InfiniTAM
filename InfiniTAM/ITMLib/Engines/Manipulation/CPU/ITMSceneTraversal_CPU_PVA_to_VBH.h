//  ================================================================
//  Created by Gregory Kramida on 8/29/19.
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
#include <unordered_set>

//local
#include "ITMSceneTraversal_CPU_AuxilaryFunctions.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib {


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		int totalPrimaryVoxelCount = primaryVolume->index.getVolumeSize().x * primaryVolume->index.getVolumeSize().y *
		                             primaryVolume->index.getVolumeSize().z;
		const ITMVoxelBlockHash::IndexData* hashTable = secondaryVolume->index.getIndexData();
		const ITMPlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.getIndexData();
		Vector3i pvaOffset = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = pvaOffset + pvaSize; // open last traversal bound (this voxel doesn't get processed)

		//determine the opposite corners of the 3d extent, [start, end), in voxel hash blocks, to traverse.
		Vector3i startBlockCoords, endBlockCoords;
		pointToVoxelBlockPos(pvaOffset, startBlockCoords);
		pointToVoxelBlockPos(endVoxel - Vector3i(1), endBlockCoords);
		endBlockCoords += Vector3i(1);
		std::unordered_set<int> coveredBlockHashes;
		volatile bool foundMismatch = false;

		//TODO: inner loop traversal should go over *entire VBH block*, not just the portion of it that is spanned by PVA.
		// Case in point: the VBH volume has an allocated hash block that spans somewhat outside the PVA extent.
		//                This VBH volume has a voxel altered within the region of this block that is outside of the PVA
		//                extent. This voxel ideally constitutes a mismatch, but this case is not covered here.
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int zVoxelBlock = startBlockCoords.z; zVoxelBlock < endBlockCoords.z; zVoxelBlock++) {
			if (foundMismatch) continue;
			int zVoxelStart = std::max(zVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.z);
			int zVoxelEnd = std::min((zVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.z);
			for (int yVoxelBlock = startBlockCoords.y; yVoxelBlock < endBlockCoords.y && !foundMismatch; yVoxelBlock++) {
				int yVoxelStart = std::max(yVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.y);
				int yVoxelEnd = std::min((yVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.y);
				for (int xVoxelBlock = startBlockCoords.x; xVoxelBlock < endBlockCoords.x && !foundMismatch; xVoxelBlock++) {
					int xVoxelStart = std::max(xVoxelBlock * SDF_BLOCK_SIZE, pvaOffset.x);
					int xVoxelEnd = std::min((xVoxelBlock + 1) * SDF_BLOCK_SIZE, endVoxel.x);
					Vector3s voxelBlockCoords(xVoxelBlock, yVoxelBlock, zVoxelBlock);
					int hash;
					bool voxelBlockAllocated = false;
					TVoxelPrimary* localSecondaryVoxelBlock = nullptr;
					// see if the voxel block is allocated at all in VBH-indexed volume
					if (FindHashAtPosition(hash, voxelBlockCoords, hashTable)) {
						voxelBlockAllocated = true;
						localSecondaryVoxelBlock = &(secondaryVoxels[hashTable[hash].ptr *
						                                             (SDF_BLOCK_SIZE3)]);
#ifdef WITH_OPENMP
#pragma omp critical
#endif
						{
							coveredBlockHashes.insert(hash);
						};
					}
					// traverse the current voxel block volume to see if PVA-indexed volume's voxels satisfy the boolean
					// functional when matched with corresponding voxels from the VBH-indexed volume.
					// If the block is not allocated in the VBH volume at all but the PVA voxel is modified,
					// short-circuit to "false"
					for (int z = zVoxelStart; z < zVoxelEnd; z++) {
						for (int y = yVoxelStart; y < yVoxelEnd; y++) {
							for (int x = xVoxelStart; x < xVoxelEnd; x++) {
								Vector3i voxelPosition = Vector3i(x, y, z);
								Vector3i voxelPositionSansOffset = voxelPosition - pvaOffset;
								int linearIndex = voxelPositionSansOffset.x + voxelPositionSansOffset.y * pvaSize.x +
								                  voxelPositionSansOffset.z * pvaSize.x * pvaSize.y;
								int locId = voxelPosition.x + (voxelPosition.y - voxelBlockCoords.x) * SDF_BLOCK_SIZE +
								            (voxelPosition.z - voxelBlockCoords.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE -
								            voxelBlockCoords.z * SDF_BLOCK_SIZE3;
								TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
								if (voxelBlockAllocated) {
									TVoxelSecondary& secondaryVoxel = localSecondaryVoxelBlock[locId];
									if (!(functor(primaryVoxel, secondaryVoxel))) {
										foundMismatch = true;
										break;
									}
								} else {
									if (isAltered(primaryVoxel)) {
										foundMismatch = true;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
		if (foundMismatch) {
			return false;
		}
		int totalHashEntryCount = secondaryVolume->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			if (foundMismatch) continue;
			ITMHashEntry secondaryHashEntry = hashTable[hash];
			if (secondaryHashEntry.ptr < 0 || coveredBlockHashes.find(hash) != coveredBlockHashes.end()) continue;
			//found allocated hash block that wasn't spanned by the volume, check to make sure it wasn't altered
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			if (isVoxelBlockAltered(secondaryVoxelBlock)) {
				foundMismatch = true;
			}
		}
		return !foundMismatch;
	}


	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location, which ignores unallocated spaces in either scene.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated. Areas where either or both of the scenes don't have allocated voxels are
	 * ignored, even if only one of the volumes does, in fact, have potentially-altered voxels there.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume <TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		volatile bool foundMismatch = false;
		int totalHashEntryCount = secondaryVolume->index.noTotalEntries;
		TVoxelSecondary* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TVoxelPrimary* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		const ITMVoxelBlockHash::IndexData* hashTable = secondaryVolume->index.getIndexData();
		const ITMPlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.getIndexData();
		Vector3i startVoxel = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = startVoxel + pvaSize; // open last traversal bound (this voxel doesn't get processed)
		Vector6i pvaBounds(startVoxel.x, startVoxel.y, startVoxel.z, endVoxel.x, endVoxel.y, endVoxel.z);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			if (foundMismatch) continue;
			ITMHashEntry secondaryHashEntry = hashTable[hash];
			if (secondaryHashEntry.ptr < 0  ) continue;
			Vector3i hashEntryMinPoint = secondaryHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(SDF_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, pvaBounds)) {
				continue;
			}
			TVoxelSecondary* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, pvaBounds);
			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						Vector3i voxelBlockPosition = Vector3i(x, y, z);
						Vector3i voxelAbsolutePosition = hashEntryMinPoint + voxelBlockPosition;
						Vector3i voxelPositionSansOffset = voxelAbsolutePosition - startVoxel;
						int linearIndex = voxelPositionSansOffset.x + voxelPositionSansOffset.y * pvaSize.x +
						                  voxelPositionSansOffset.z * pvaSize.x * pvaSize.y;
						TVoxelSecondary& secondaryVoxel = secondaryVoxelBlock[locId];
						TVoxelPrimary& primaryVoxel = primaryVoxels[linearIndex];
						if (!(functor(primaryVoxel, secondaryVoxel))) {
							foundMismatch = true;
							break;
						}
					}
				}
			}
		}
		return !foundMismatch;
	}
};

template<typename TVoxelPrimary, typename TVoxelSecondary, typename TFunctor>
struct ITMFlipArgumentBooleanFunctor {
	ITMFlipArgumentBooleanFunctor(TFunctor functor) : functor(functor) {}

	bool operator()(TVoxelPrimary& voxelPrimary, TVoxelSecondary& voxelSecondary) {
		return functor(voxelSecondary, voxelPrimary);
	}

	TFunctor functor;
};

template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume <TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary,TVoxelSecondary,TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
		DualVoxelTraversal_AllTrue(secondaryVolume, primaryVolume, flipFunctor);

	}


	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			ITMVoxelVolume <TVoxelPrimary, ITMVoxelBlockHash>* primaryVolume,
			ITMVoxelVolume <TVoxelSecondary, ITMPlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary,TVoxelSecondary,TFunctor> flipFunctor(functor);
		return ITMDualSceneTraversalEngine<TVoxelSecondary, TVoxelPrimary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
		DualVoxelTraversal_AllTrue_AllocatedOnly(secondaryVolume, primaryVolume, flipFunctor);
	}
};
} // namespace ITMLib