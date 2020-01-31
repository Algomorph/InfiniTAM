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
#include "../Shared/VolumeTraversal_Shared.h"
#include "../Interface/TwoVolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/Analytics/IsAltered.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Utils/Geometry/GeometryBooleanOperations.h"

namespace ITMLib {


template<typename TArrayVoxel, typename THashVoxel>
class TwoVolumeTraversalEngine<TArrayVoxel, THashVoxel, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU> {
private:

	template<typename TTwoVoxelBooleanFunctor, typename TTwoVoxelAndPositionPredicate,
			typename TArrayVoxelPredicate, typename THashVoxelPredicate>
	inline static bool
	DualVoxelTraversal_AllTrue_Generic(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* arrayVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* hashVolume,
			TTwoVoxelBooleanFunctor& twoVoxelBooleanFunctor,
			TTwoVoxelAndPositionPredicate&& twoVoxelAndPositionPredicate,
			TArrayVoxelPredicate&& arrayVoxelAlteredCheckPredicate,
			THashVoxelPredicate&& hashVoxelAlteredCheckPredicate,
			bool verbose
	) {


		const int hashEntryCount = hashVolume->index.hashEntryCount;
		ITMHashEntry* hashTable = hashVolume->index.GetIndexData();

		TArrayVoxel* arrayVoxels = arrayVolume->localVBA.GetVoxelBlocks();
		THashVoxel* hashVoxels = hashVolume->localVBA.GetVoxelBlocks();

		PlainVoxelArray::GridAlignedBox* arrayInfo = arrayVolume->index.GetIndexData();
		Vector3i arrayMinVoxels = arrayInfo->offset;
		Vector3i arrayMaxVoxels = arrayInfo->offset + arrayInfo->size;

		volatile bool mismatchFound = false;

		// *** find voxel blocks in hash not spanned by array and inspect if any are altered ***
		// *** also find blocks partially overlapping array, and inspect if those outside array are altered and those inside match ***
#ifdef WITH_OPENMP
#pragma omp parallel for //default(none), shared(mismatchFound, hashTable, arrayMinVoxels, arrayMaxVoxels, verbose)
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			if (mismatchFound) continue;
			ITMHashEntry& hashEntry = hashTable[hashCode];
			if (hashEntry.ptr < 0) {
				continue;
			}
			Vector3i blockMinVoxels = hashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i blockMaxVoxels = blockMinVoxels + Vector3i(VOXEL_BLOCK_SIZE);
			if (blockMaxVoxels.x < arrayMinVoxels.x || blockMinVoxels.x > arrayMaxVoxels.x ||
			    blockMaxVoxels.y < arrayMinVoxels.y || blockMinVoxels.y > arrayMaxVoxels.y ||
			    blockMaxVoxels.z < arrayMinVoxels.z || blockMinVoxels.z > arrayMaxVoxels.z) {
				if (verbose) {
					if (isVoxelBlockAlteredPredicate(hashVoxels + hashEntry.ptr * VOXEL_BLOCK_SIZE3,
					                                 arrayVoxelAlteredCheckPredicate, true,
					                                 "Hash voxel unmatched in array: ", hashEntry.pos, hashCode)) {
						mismatchFound = true;
					}
				} else {
					if (isVoxelBlockAlteredPredicate(hashVoxels + hashEntry.ptr * VOXEL_BLOCK_SIZE3,
					                                 arrayVoxelAlteredCheckPredicate)) {
						mismatchFound = true;
					}
				}
			} else if (blockMaxVoxels.x > arrayMaxVoxels.x || blockMinVoxels.x < arrayMinVoxels.x ||
			           blockMaxVoxels.y > arrayMaxVoxels.y || blockMinVoxels.y < arrayMinVoxels.y ||
			           blockMaxVoxels.z > arrayMaxVoxels.z || blockMinVoxels.z < arrayMinVoxels.z
					) {
				THashVoxel* voxelBlock = hashVoxels + hashEntry.ptr * VOXEL_BLOCK_SIZE3;
				for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
					THashVoxel& hashVoxel = voxelBlock[linearIndexInBlock];
					if (std::forward<THashVoxelPredicate>(hashVoxelAlteredCheckPredicate)(hashVoxel) &&
					    isAltered(hashVoxel)) {
						Vector3i voxelPosition =
								ComputePositionVectorFromLinearIndex_VoxelBlockHash(hashEntry.pos, linearIndexInBlock);
						if (isPointInBounds(voxelPosition, *arrayInfo)) {
							int linearIndexInArray = ComputeLinearIndexFromPosition_PlainVoxelArray(arrayInfo,
							                                                                        voxelPosition);
							TArrayVoxel& arrayVoxel = arrayVoxels[linearIndexInArray];
							if (!std::forward<TTwoVoxelAndPositionPredicate>(twoVoxelAndPositionPredicate)(
									twoVoxelBooleanFunctor, arrayVoxel, hashVoxel,
									voxelPosition)) {
								mismatchFound = true;
							}
						} else {
							if (verbose) {
								isAltered_VerbosePositionHash(hashVoxel, voxelPosition, hashCode, hashEntry.pos,
								                              "Hash voxel unmatched in array: ");
							}
							mismatchFound = true;
						}
					}
				}
			}
		}
		if (mismatchFound) return false;


		int voxelCount = arrayVolume->index.GetVoxelBlockSize();
#ifndef WITH_OPENMP
		VoxelBlockHash::IndexCache cache;
#endif

		// *** Checks whether voxels on the margin of the array that don't overlap any voxel blocks are altered ***
		auto marginExtentHasMismatch = [&](const Extent3D& extent) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
			for (int z = extent.min_z; z < extent.max_z; z++) {
				if (mismatchFound) continue;
				for (int y = extent.min_y; y < extent.max_y; y++) {
					for (int x = extent.min_x; x < extent.max_x; x++) {
						Vector3i position(x, y, z);
						int linearArrayIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(arrayInfo, position);
						int vmIndex;
#ifndef WITH_OPENMP
						int hashVoxelIndex = findVoxel(hashTable, position, vmIndex, cache);
#else
						int hashVoxelIndex = findVoxel(hashTable, position, vmIndex);
#endif
						// no hash block intersecting array at this point, yet voxel is altered: fail
						TArrayVoxel& arrayVoxel = arrayVoxels[linearArrayIndex];
						if (verbose) {
							if (hashVoxelIndex == -1
							    && std::forward<TArrayVoxelPredicate>(arrayVoxelAlteredCheckPredicate)(arrayVoxel)
							    && isAltered_VerbosePosition(arrayVoxel, position, "Array voxel not matched in hash: ")) {
								mismatchFound = true;
							}
						} else {
							if (hashVoxelIndex == -1
							    && std::forward<TArrayVoxelPredicate>(arrayVoxelAlteredCheckPredicate)(arrayVoxel)
							    && isAltered(arrayVoxel)) {
								mismatchFound = true;
							}
						}
					}
				}
			}
			return mismatchFound;
		};

		// *** checks all voxels inside array, return false if one's altered but unallocated in the hash or if
		// the twoVoxelBooleanFunctor on it and the corresponding one from the hash-block volume returns false ***

		auto centralExtentHasMismatch = [&](const Extent3D& extent) {
			Extent3D hashBlockExtent = extent / VOXEL_BLOCK_SIZE;
#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
			for (int zBlock = hashBlockExtent.min_z; zBlock < hashBlockExtent.max_z; zBlock++) {
				if (mismatchFound) continue;
				int zVoxelStart = zBlock * VOXEL_BLOCK_SIZE, zVoxelEnd = zVoxelStart + VOXEL_BLOCK_SIZE;
				for (int yBlock = hashBlockExtent.min_y; yBlock < hashBlockExtent.max_y; yBlock++) {
					if (mismatchFound) continue;
					int yVoxelStart = yBlock * VOXEL_BLOCK_SIZE, yVoxelEnd = yVoxelStart + VOXEL_BLOCK_SIZE;
					for (int xBlock = hashBlockExtent.min_x; xBlock < hashBlockExtent.max_x; xBlock++) {
						if (mismatchFound) continue;
						int xVoxelStart = xBlock * VOXEL_BLOCK_SIZE, xVoxelEnd = xVoxelStart + VOXEL_BLOCK_SIZE;
						Vector3s voxelBlockCoords(xBlock, yBlock, zBlock);
						int hash;
						TArrayVoxel* hashBlockVoxels = nullptr;
						if (FindHashAtPosition(hash, voxelBlockCoords, hashTable)) {
							hashBlockVoxels = hashVoxels + hashTable[hash].ptr * VOXEL_BLOCK_SIZE3;
							// traverse the current voxel block volume to see if PVA volume's voxels satisfy the
							// boolean predicate twoVoxelBooleanFunctor when matched with corresponding voxels from the VBH volume.
							int idWithinBlock = 0;
							for (int zAbsolute = zVoxelStart; zAbsolute < zVoxelEnd; zAbsolute++) {
								for (int yAbsolute = yVoxelStart; yAbsolute < yVoxelEnd; yAbsolute++) {
									for (int xAbsolute = xVoxelStart;
									     xAbsolute < xVoxelEnd; xAbsolute++, idWithinBlock++) {
										Vector3i positionAbsolute(xAbsolute, yAbsolute, zAbsolute);
										int linearArrayIndex =
												ComputeLinearIndexFromPosition_PlainVoxelArray(arrayInfo, positionAbsolute);
										TArrayVoxel& arrayVoxel = arrayVoxels[linearArrayIndex];
										THashVoxel& hashVoxel = hashBlockVoxels[idWithinBlock];
										if (!std::forward<TTwoVoxelAndPositionPredicate>(twoVoxelAndPositionPredicate)(
												twoVoxelBooleanFunctor, arrayVoxel, hashVoxel, positionAbsolute)) {
											mismatchFound = true;
										}
									}
								}
							}
						} else {
							// If the block is not allocated in the VBH volume at all but the PVA voxel is modified,
							// short-circuit to "false"
							for (int zAbsolute = zVoxelStart; zAbsolute < zVoxelEnd; zAbsolute++) {
								for (int yAbsolute = yVoxelStart; yAbsolute < yVoxelEnd; yAbsolute++) {
									for (int xAbsolute = xVoxelStart; xAbsolute < xVoxelEnd; xAbsolute++) {
										Vector3i positionAbsolute(xAbsolute, yAbsolute, zAbsolute);
										int linearArrayIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(arrayInfo,
										                                                                      positionAbsolute);
										int vmIndex;
										// no hash block intersecting array at this point, yet voxel is altered: fail
										TArrayVoxel& arrayVoxel = arrayVoxels[linearArrayIndex];
										if (verbose) {
											if (std::forward<TArrayVoxelPredicate>(arrayVoxelAlteredCheckPredicate)(
													arrayVoxel)
											    && isAltered_VerbosePosition(arrayVoxel, positionAbsolute,
											                                 "Array voxel not matched in hash: ")) {
												mismatchFound = true;
											}
										} else {
											if (std::forward<TArrayVoxelPredicate>(arrayVoxelAlteredCheckPredicate)(
													arrayVoxel) && isAltered(arrayVoxel)) {
												mismatchFound = true;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			return mismatchFound;
		};

		// *** compute extents ***
		Extent3D centralExtent;
		std::vector<Extent3D> borderExtents = ComputeBoxSetOfHashAlignedCenterAndNonHashBlockAlignedArrayMargins(
				*arrayInfo, centralExtent);
		for (auto& extent : borderExtents) {
			if (marginExtentHasMismatch(extent)) return false;
		}
		return !centralExtentHasMismatch(centralExtent);
	}

	template<typename TFunctor, typename TFunctionCall>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* primaryVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* secondaryVolume,
			TFunctor& functor, TFunctionCall&& functionCall) {
		volatile bool foundMismatch = false;
		int totalHashEntryCount = secondaryVolume->index.hashEntryCount;
		THashVoxel* secondaryVoxels = secondaryVolume->localVBA.GetVoxelBlocks();
		TArrayVoxel* primaryVoxels = primaryVolume->localVBA.GetVoxelBlocks();
		const VoxelBlockHash::IndexData* hashTable = secondaryVolume->index.GetIndexData();
		const PlainVoxelArray::IndexData* pvaIndexData = primaryVolume->index.GetIndexData();
		Vector3i startVoxel = pvaIndexData->offset;
		Vector3i pvaSize = pvaIndexData->size;
		Vector3i endVoxel = startVoxel + pvaSize; // open last traversal bound (this voxel doesn't get processed)
		Vector6i pvaBounds(startVoxel.x, startVoxel.y, startVoxel.z, endVoxel.x, endVoxel.y, endVoxel.z);

#ifdef WITH_OPENMP
#pragma omp parallel for default(shared)
#endif
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			if (foundMismatch) continue;
			ITMHashEntry secondaryHashEntry = hashTable[hash];
			if (secondaryHashEntry.ptr < 0) continue;
			Vector3i voxelBlockMinPoint = secondaryHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = voxelBlockMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(voxelBlockMinPoint, hashEntryMaxPoint, pvaBounds)) {
				continue;
			}
			THashVoxel* secondaryVoxelBlock = &(secondaryVoxels[secondaryHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			Vector6i localBounds = computeLocalBounds(voxelBlockMinPoint, hashEntryMaxPoint, pvaBounds);
			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelCoordinateWithinBlock = Vector3i(x, y, z);
						Vector3i voxelAbsolutePosition = voxelBlockMinPoint + voxelCoordinateWithinBlock;
						Vector3i voxelPositionSansOffset = voxelAbsolutePosition - startVoxel;
						int linearIndex = voxelPositionSansOffset.x + voxelPositionSansOffset.y * pvaSize.x +
						                  voxelPositionSansOffset.z * pvaSize.x * pvaSize.y;
						THashVoxel& secondaryVoxel = secondaryVoxelBlock[locId];
						TArrayVoxel& primaryVoxel = primaryVoxels[linearIndex];

						if (!std::forward<TFunctionCall>(functionCall)(functor, primaryVoxel, secondaryVoxel,
						                                               voxelAbsolutePosition)) {
							foundMismatch = true;
							break;
						}
					}
				}
			}
		}
		return !foundMismatch;

	}

public:

	static inline
	std::vector<Extent3D>
	ComputeBoxSetOfHashAlignedCenterAndNonHashBlockAlignedArrayMargins(
			const ITMLib::PlainVoxelArray::GridAlignedBox& arrayInfo, Extent3D& centerExtent) {

		//TODO: code can probably be condensed throughout to act on a per-dimension basis using Vector indexing, 
		// but not sure if code clarity will be preserved

		Vector3i arrayExtentMin = arrayInfo.offset;

		// compute how many voxels on each side of the array extend past the first hash block fully inside the array.
		// these are the margin thicknesses, in voxels
		int margin_near_z = -arrayExtentMin.z % VOXEL_BLOCK_SIZE + (arrayExtentMin.z <= 0 ? 0 : VOXEL_BLOCK_SIZE);
		int margin_near_y = -arrayExtentMin.y % VOXEL_BLOCK_SIZE + (arrayExtentMin.y <= 0 ? 0 : VOXEL_BLOCK_SIZE);
		int margin_near_x = -arrayExtentMin.x % VOXEL_BLOCK_SIZE + (arrayExtentMin.x <= 0 ? 0 : VOXEL_BLOCK_SIZE);
		Vector3i arrayExtentMax = arrayExtentMin + arrayInfo.size;
		int margin_far_z = arrayExtentMax.z % VOXEL_BLOCK_SIZE + (arrayExtentMax.z <= 0 ? VOXEL_BLOCK_SIZE : 0);
		int margin_far_y = arrayExtentMax.y % VOXEL_BLOCK_SIZE + (arrayExtentMax.y <= 0 ? VOXEL_BLOCK_SIZE : 0);
		int margin_far_x = arrayExtentMax.x % VOXEL_BLOCK_SIZE + (arrayExtentMax.x <= 0 ? VOXEL_BLOCK_SIZE : 0);

		// use the margin thicknesses to construct the 6 blocks, one for each face. The boxes should not overlap.
		int margin_near_x_end = arrayExtentMin.x + margin_near_x;
		int margin_far_x_start = arrayExtentMax.x - margin_far_x;
		int margin_near_y_end = arrayExtentMin.y + margin_near_y;
		int margin_far_y_start = arrayExtentMax.y - margin_near_y;
		int margin_near_z_end = arrayExtentMin.z + margin_near_z;
		int margin_far_z_start = arrayExtentMax.z - margin_near_z;

		std::vector<Extent3D> allExtents = {
				Extent3D{arrayExtentMin.x, arrayExtentMin.y, arrayExtentMin.z,
				         margin_near_x_end, arrayExtentMax.y, arrayExtentMax.z},
				Extent3D{margin_far_x_start, arrayExtentMin.y, arrayExtentMin.z,
				         arrayExtentMax.x, arrayExtentMax.y, arrayExtentMax.z},

				Extent3D{margin_near_x_end, arrayExtentMin.y, arrayExtentMin.z,
				         margin_far_x_start, margin_near_y_end, arrayExtentMax.z},
				Extent3D{margin_near_x_end, margin_far_y_start, arrayExtentMin.z,
				         margin_far_x_start, arrayExtentMax.y, arrayExtentMax.z},

				Extent3D{margin_near_x_end, margin_near_y_end, arrayExtentMin.z,
				         margin_far_x_start, margin_far_z_start, margin_near_z_end},
				Extent3D{margin_near_x_end, margin_near_y_end, margin_far_z_start,
				         margin_far_x_start, margin_far_z_start, arrayExtentMax.z},
		};

		std::array<int, 6> margins = {
				margin_near_x, margin_far_x,
				margin_near_y, margin_far_y,
				margin_near_z, margin_far_z
		};

		std::vector<Extent3D> nonZeroExtents;
		for (int i_margin = 0; i_margin < margins.size(); i_margin++) {
			if (margins[i_margin] > 0) nonZeroExtents.push_back(allExtents[i_margin]);
		}
		// add central extent
		centerExtent = {margin_near_x_end, margin_near_y_end, margin_near_z_end,
		                margin_far_x_start, margin_far_y_start, margin_far_z_start};
		return nonZeroExtents;
	}

	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param arrayVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param hashVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* arrayVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* hashVolume,
			TFunctor& functor, bool verbose) {
		return DualVoxelTraversal_AllTrue_Generic(arrayVolume, hashVolume, functor, []
				(TFunctor& functor1, TArrayVoxel& voxelArray,
				 THashVoxel& voxelHash, const Vector3i& voxelPosition) {
			return functor1(voxelArray, voxelHash);
		}, [](TArrayVoxel& arrayVoxel) { return true; }, [](THashVoxel& hashVoxel) { return true; }, verbose);
	}


	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* primaryVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* secondaryVolume,
			TFunctor& functor, bool verbose) {
		return DualVoxelTraversal_AllTrue_Generic(primaryVolume, secondaryVolume, functor, []
				(TFunctor& functor, TArrayVoxel& voxelPrimary, THashVoxel& voxelSecondary,
				 const Vector3i& voxelPosition) {
			return functor(voxelPrimary, voxelSecondary, voxelPosition);
		}, [](TArrayVoxel& arrayVoxel) { return true; }, [](THashVoxel& hashVoxel) { return true; }, verbose);
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_MatchingFlags(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* arrayVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* hashVolume,
			VoxelFlags flags,
			TFunctor& functor, bool verbose) {
		return DualVoxelTraversal_AllTrue_Generic(
				arrayVolume, hashVolume, functor,
				[&flags](TFunctor& functor1,
				         TArrayVoxel& voxelArray, THashVoxel& hashVoxel, const Vector3i& voxelPosition) {
					return (voxelArray.flags != flags && hashVoxel.flags != flags) || functor1(voxelArray, hashVoxel);
				},
				[&flags](TArrayVoxel& arrayVoxel) { return arrayVoxel.flags == flags; },
				[&flags](THashVoxel& hashVoxel) { return hashVoxel.flags == flags; }, verbose);
	}


	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue_MatchingFlags(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* primaryVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* secondaryVolume,
			VoxelFlags flags, TFunctor& functor, bool verbose) {
		return DualVoxelTraversal_AllTrue_Generic(
				primaryVolume, secondaryVolume, functor,
				[&flags](TFunctor& functor,
				         TArrayVoxel& arrayVoxel, THashVoxel& hashVoxel, const Vector3i& voxelPosition) {
					return (arrayVoxel.flags != flags && hashVoxel.flags != flags) ||
					       functor(arrayVoxel, hashVoxel, voxelPosition);
				},
				[&flags](TArrayVoxel& arrayVoxel) { return arrayVoxel.flags == flags; },
				[&flags](THashVoxel& hashVoxel) { return hashVoxel.flags == flags; }, verbose);
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
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* primaryVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(primaryVolume, secondaryVolume, functor, []
				(TFunctor& functor, TArrayVoxel& voxelPrimary, THashVoxel& voxelSecondary,
				 const Vector3i& voxelPosition) {
			return functor(voxelPrimary, voxelSecondary);
		});
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
	 * \param functor a function object accepting two voxels and their mutually shared position by reference as arguments
	 * and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelPositionTraversal_AllTrue_AllocatedOnly(
			VoxelVolume <TArrayVoxel, PlainVoxelArray>* primaryVolume,
			VoxelVolume <THashVoxel, VoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		return DualVoxelTraversal_AllTrue_AllocatedOnly_Generic(primaryVolume, secondaryVolume, functor, []
				(TFunctor& functor, TArrayVoxel& voxelPrimary, THashVoxel& voxelSecondary,
				 Vector3i& voxelPosition) {
			return functor(voxelPrimary, voxelSecondary, voxelPosition);
		});
	}


};


template<typename TVoxelPrimary, typename TVoxelSecondary>
class TwoVolumeTraversalEngine<TVoxelPrimary, TVoxelSecondary, VoxelBlockHash, PlainVoxelArray, MEMORYDEVICE_CPU> {
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
			VoxelVolume <TVoxelPrimary, VoxelBlockHash>* primaryVolume,
			VoxelVolume <TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor, bool verbose) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU>::
		DualVoxelTraversal_AllTrue(secondaryVolume, primaryVolume, flipFunctor, verbose);

	}


	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue_AllocatedOnly(
			VoxelVolume <TVoxelPrimary, VoxelBlockHash>* primaryVolume,
			VoxelVolume <TVoxelSecondary, PlainVoxelArray>* secondaryVolume,
			TFunctor& functor) {
		ITMFlipArgumentBooleanFunctor<TVoxelPrimary, TVoxelSecondary, TFunctor> flipFunctor(functor);
		return TwoVolumeTraversalEngine<TVoxelSecondary, TVoxelPrimary, PlainVoxelArray, VoxelBlockHash, MEMORYDEVICE_CPU>::
		DualVoxelTraversal_AllTrue_AllocatedOnly(secondaryVolume, primaryVolume, flipFunctor);
	}
};
} // namespace ITMLib