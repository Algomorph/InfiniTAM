//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../Common/CommonFunctors.h"
#include "../../Common/AllocationTempData.h"
#include "../../../Utils/ITMMath.h"
#include "../../../Utils/ITMPixelUtils.h"
#include "../../../Utils/ITMVoxelFlags.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../Utils/Geometry/ITMIntersectionChecks.h"

#ifdef __CUDACC__
#include "../../../Utils/ITMCUDAUtils.h"
#include "../../Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#endif

using namespace ITMLib;




/**
 * \brief Determines whether the hash block at the specified block position needs it's voxels to be allocated, as well
 * as whether they should be allocated in the excess list or the ordered list of the hash table.
 * If any of these are true, marks the corresponding entry in \param hashEntryStates
 * \param[in,out] hashEntryStates  array where to set the allocation type at final hashIdx index
 * \param[in,out] hashBlockCoordinates  array block coordinates for the new hash blocks at final hashIdx index
 * \param[in,out] hashCode  takes in original index assuming coords, i.e. \refitem HashCodeFromBlockPosition(\param desiredHashBlockPosition),
 * returns final index of the hash block to be allocated (may be updated based on hash closed chaining)
 * \param[in] desiredHashBlockPosition  position of the hash block to check / allocate
 * \param[in] hashTable  hash table with existing blocks
 * \param[in] collisionDetected set to true if a block with the same hashcode has already been marked for allocation ( a collision occured )
 * \return true if the block needs allocation, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool MarkAsNeedingAllocationIfNotFound(ITMLib::HashEntryAllocationState* hashEntryStates,
                                              Vector3s* hashBlockCoordinates, int& hashCode,
                                              const CONSTPTR(Vector3s)& desiredHashBlockPosition,
                                              const CONSTPTR(ITMHashEntry)* hashTable, bool& collisionDetected) {

	ITMHashEntry hashEntry = hashTable[hashCode];
	//check if hash table contains entry

	if (!(IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1)) {

		auto setHashEntryState = [&](HashEntryAllocationState state) {
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
			if (atomicCAS((char*) hashEntryStates + hashCode,
					  (char) ITMLib::NEEDS_NO_CHANGE,
					  (char) state) != ITMLib::NEEDS_NO_CHANGE){
			if (IS_EQUAL3(hashBlockCoordinates[hashCode], desiredHashBlockPosition)) return false;
			//hash code already marked for allocation, but at different coordinates, cannot allocate
			collisionDetected = true;
			return false;
		} else {
			hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
			return true;
		}
#else
			//TODO: come up with an atomics-based solution for OpenMP
			bool success = false;
#ifdef WITH_OPENMP
#pragma omp critical
#endif
			{
				//single-threaded version
				if (hashEntryStates[hashCode] != ITMLib::NEEDS_NO_CHANGE) {
					if (!IS_EQUAL3(hashBlockCoordinates[hashCode], desiredHashBlockPosition)) {
						//hash code already marked for allocation, but at different coordinates, cannot allocate
						collisionDetected = true;
					}
					success = false;
				} else {
					hashEntryStates[hashCode] = state;
					hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
					success = true;
				}
			}
			return success;
#endif
		};
		if (hashEntry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hashEntry.offset >= 1) {
				hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
				hashEntry = hashTable[hashCode];

				if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
					return false;
				}
			}
			return setHashEntryState(ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST);
		}
		return setHashEntryState(ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST);
	}
	// already have hash block, no allocation needed
	return false;

};


_CPU_AND_GPU_CODE_ inline void
MarkForAllocationAndSetVisibilityTypeIfNotFound(ITMLib::HashEntryAllocationState* hashEntryStates,
                                                Vector3s* hashBlockCoordinates,
                                                HashBlockVisibility* blockVisibilityTypes,
                                                Vector3s desiredHashBlockPosition,
                                                const CONSTPTR(ITMHashEntry)* hashTable, bool& collisionDetected) {

	int hashCode = HashCodeFromBlockPosition(desiredHashBlockPosition);

	ITMHashEntry hashEntry = hashTable[hashCode];

	//check if hash table contains entry
	if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
		//entry has been streamed out but is visible or in memory and visible
		blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? STREAMED_OUT_AND_VISIBLE
		                                                       : IN_MEMORY_AND_VISIBLE;
		return;
	}

	HashEntryAllocationState allocationState = NEEDS_ALLOCATION_IN_ORDERED_LIST;
	if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
	{
		while (hashEntry.offset >= 1) {
			hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
			hashEntry = hashTable[hashCode];

			if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
				//entry has been streamed out but is visible or in memory and visible
				blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? STREAMED_OUT_AND_VISIBLE
				                                                       : IN_MEMORY_AND_VISIBLE;
				return;
			}
		}
		allocationState = NEEDS_ALLOCATION_IN_EXCESS_LIST;
	}

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
	if (atomicCAS((char*) hashEntryStates + hashCode,
				  (char) ITMLib::NEEDS_NO_CHANGE,
				  (char) allocationState) != ITMLib::NEEDS_NO_CHANGE) {
		if (IS_EQUAL3(hashBlockCoordinates[hashCode], desiredHashBlockPosition)) return;
		collisionDetected = true;
	} else {
		//needs allocation
		if (allocationState == NEEDS_ALLOCATION_IN_ORDERED_LIST)
			blockVisibilityTypes[hashCode] = HashBlockVisibility::IN_MEMORY_AND_VISIBLE; //new entry is visible
		hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
	}
#else
#if defined(WITH_OPENMP)
#pragma omp critical
#endif
	{
		if (hashEntryStates[hashCode] != ITMLib::NEEDS_NO_CHANGE) {
			collisionDetected = true;
		} else {
			//needs allocation
			hashEntryStates[hashCode] = allocationState;
			if (allocationState == NEEDS_ALLOCATION_IN_ORDERED_LIST)
				blockVisibilityTypes[hashCode] = HashBlockVisibility::IN_MEMORY_AND_VISIBLE; //new entry is visible
			hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
		}
	}
#endif
}


template<typename TWarp, typename TVoxel, WarpType TWarpType>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			VoxelVolume<TVoxel, VoxelBlockHash>* sourceVolume,
			VoxelVolume<TVoxel, VoxelBlockHash>* volumeToAllocate,
			Vector3s* allocationBlockCoords,
			HashEntryAllocationState* warpedEntryAllocationStates) :

			collisionDetected(false),

			targetTSDFScene(volumeToAllocate),
			targetTSDFVoxels(volumeToAllocate->localVBA.GetVoxelBlocks()),
			targetTSDFHashEntries(volumeToAllocate->index.GetEntries()),
			targetTSDFCache(),

			sourceTSDFScene(sourceVolume),
			sourceTSDFVoxels(sourceVolume->localVBA.GetVoxelBlocks()),
			sourceTSDFHashEntries(sourceVolume->index.GetEntries()),
			sourceTSDFCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationStates(warpedEntryAllocationStates) {}

	_CPU_AND_GPU_CODE_
	inline
	void operator()(TWarp& warpVoxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {

		Vector3f warpVector = ITMLib::WarpVoxelStaticFunctor<TWarp, TWarpType>::GetWarp(warpVoxel);
		Vector3f warpedPosition = warpVector + TO_FLOAT3(voxelPosition);
		Vector3i warpedPositionTruncated = warpedPosition.toInt();

		// perform lookup in source volume
		int vmIndex;
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
														warpedPositionTruncated,
														vmIndex, sourceTSDFCache);
#else //don't use cache when multithreading!
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
		                                                warpedPositionTruncated,
		                                                vmIndex);
#endif

		int targetBlockHash = HashCodeFromBlockPosition(hashBlockPosition);

		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationStates, allocationBlockCoords, targetBlockHash,
		                                  hashBlockPosition, targetTSDFHashEntries, collisionDetected);
	}

	bool collisionDetected;

private:


	VoxelVolume<TVoxel, VoxelBlockHash>* targetTSDFScene;
	TVoxel* targetTSDFVoxels;
	ITMHashEntry* targetTSDFHashEntries;
	VoxelBlockHash::IndexCache targetTSDFCache;

	VoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDFScene;
	TVoxel* sourceTSDFVoxels;
	ITMHashEntry* sourceTSDFHashEntries;
	VoxelBlockHash::IndexCache sourceTSDFCache;

	Vector3s* allocationBlockCoords;
	HashEntryAllocationState* warpedEntryAllocationStates;
};


_CPU_AND_GPU_CODE_ inline int getIncrementCount(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}


_CPU_AND_GPU_CODE_ inline void
prepareForAllocationFromDepthAndTsdf(ITMLib::HashEntryAllocationState* hashEntryStates,
                                     HashBlockVisibility* blockVisibilityTypes, int x, int y,
                                     Vector3s* blockCoords, const CONSTPTR(float)* depth, Matrix4f invertedCameraPose,
                                     Vector4f invertedCameraProjectionParameters, float surface_distance_cutoff,
                                     Vector2i imgSize, float oneOverVoxelBlockSize_Meters,
                                     const CONSTPTR(ITMHashEntry)* hashTable, float viewFrustum_min,
                                     float viewFrustum_max,
                                     bool& collisionDetected) {
	float depth_measure;
	int stepCount;
	Vector4f pt_camera_f;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
	    (depth_measure - surface_distance_cutoff) < viewFrustum_min ||
	    (depth_measure + surface_distance_cutoff) > viewFrustum_max)
		return;


	pt_camera_f.z = depth_measure; // (orthogonal) distance to the point from the image plane (meters)
	pt_camera_f.x =
			pt_camera_f.z * ((float(x) - invertedCameraProjectionParameters.z) * invertedCameraProjectionParameters.x);
	pt_camera_f.y =
			pt_camera_f.z * ((float(y) - invertedCameraProjectionParameters.w) * invertedCameraProjectionParameters.y);

	// distance to the point along camera ray
	float norm = sqrtf(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;

	//Vector3f offset(-halfVoxelSize);
	pt_buff = pt_camera_f * (1.0f - surface_distance_cutoff / norm);
	pt_buff.w = 1.0f;
	//position along segment to march along ray in hash blocks (here -- starting point)
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	Vector3f currentCheckPosition_HashBlocks = (TO_VECTOR3(invertedCameraPose * pt_buff)) * oneOverVoxelBlockSize_Meters
	                                           + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));

	pt_buff = pt_camera_f * (1.0f + surface_distance_cutoff / norm);
	pt_buff.w = 1.0f;
	//end position of the segment to march along the ray
	Vector3f endCheckPosition_HashBlocks = (TO_VECTOR3(invertedCameraPose * pt_buff)) * oneOverVoxelBlockSize_Meters
	                                       + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::ITMSegment marchSegment(currentCheckPosition_HashBlocks, endCheckPosition_HashBlocks);

	// number of steps to take along the truncated SDF band
	stepCount = (int) std::ceil(2.0f * marchSegment.length());

	// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = marchSegment.direction / (float) (stepCount - 1);

	Vector3s previousHashBlockPosition;

	//add neighbouring blocks
	for (int i = 0; i < stepCount; i++) {
		//find block position at current step
		Vector3s currentHashBlockPosition = TO_SHORT_FLOOR3(currentCheckPosition_HashBlocks);
		int incrementCount;
		if (i > 0 && (incrementCount = getIncrementCount(currentHashBlockPosition, previousHashBlockPosition)) > 1) {
			if (incrementCount == 2) {
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					if (currentHashBlockPosition.values[iDirection] != previousHashBlockPosition.values[iDirection]) {
						Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
						potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
						if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
						                                       1.0f)) {
							MarkForAllocationAndSetVisibilityTypeIfNotFound(
									hashEntryStates,
									blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
									collisionDetected);
						}
					}
				}
			} else {
				//incrementCount == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hashEntryStates,
								blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
								collisionDetected);
					}
					potentiallyMissedBlockPosition = currentHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = previousHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hashEntryStates,
								blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
								collisionDetected);
					}
				}
			}
		}
		MarkForAllocationAndSetVisibilityTypeIfNotFound(hashEntryStates,
		                                                blockCoords,
		                                                blockVisibilityTypes, currentHashBlockPosition, hashTable,
		                                                collisionDetected);

		currentCheckPosition_HashBlocks += strideVector;
		previousHashBlockPosition = currentHashBlockPosition;
	}
}

_CPU_AND_GPU_CODE_ inline void
buildHashAllocAndVisibleTypePP(ITMLib::HashEntryAllocationState* hashEntryStates,
                               HashBlockVisibility* blockVisibilityTypes, int x, int y,
                               Vector3s* blockCoords, const CONSTPTR(float)* depth, Matrix4f invertedCameraPose,
                               Vector4f invertedCameraProjectionParameters, float surface_distance_cutoff,
                               Vector2i imgSize, float oneOverVoxelBlockSize_Meters,
                               const CONSTPTR(ITMHashEntry)* hashTable, float viewFrustum_min, float viewFrustum_max,
                               bool& collisionDetected) {
	float depth_measure;
	int stepCount;
	Vector4f pt_camera_f;

	depth_measure = depth[x + y * imgSize.x];
	if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
	    (depth_measure - surface_distance_cutoff) < viewFrustum_min ||
	    (depth_measure + surface_distance_cutoff) > viewFrustum_max)
		return;


	pt_camera_f.z = depth_measure; // (orthogonal) distance to the point from the image plane (meters)
	pt_camera_f.x =
			pt_camera_f.z * ((float(x) - invertedCameraProjectionParameters.z) * invertedCameraProjectionParameters.x);
	pt_camera_f.y =
			pt_camera_f.z * ((float(y) - invertedCameraProjectionParameters.w) * invertedCameraProjectionParameters.y);

	// distance to the point along camera ray
	float norm = sqrtf(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

	Vector4f pt_buff;

	//Vector3f offset(-halfVoxelSize);
	pt_buff = pt_camera_f * (1.0f - surface_distance_cutoff / norm);
	pt_buff.w = 1.0f;
	//position along segment to march along ray in hash blocks (here -- starting point)
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	Vector3f currentCheckPosition_HashBlocks = (TO_VECTOR3(invertedCameraPose * pt_buff)) * oneOverVoxelBlockSize_Meters
	                                           + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));

	pt_buff = pt_camera_f * (1.0f + surface_distance_cutoff / norm);
	pt_buff.w = 1.0f;
	//end position of the segment to march along the ray
	Vector3f endCheckPosition_HashBlocks = (TO_VECTOR3(invertedCameraPose * pt_buff)) * oneOverVoxelBlockSize_Meters
	                                       + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::ITMSegment marchSegment(currentCheckPosition_HashBlocks, endCheckPosition_HashBlocks);

	// number of steps to take along the truncated SDF band
	stepCount = (int) std::ceil(2.0f * marchSegment.length());

	// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = marchSegment.direction / (float) (stepCount - 1);

	Vector3s previousHashBlockPosition;

	//add neighbouring blocks
	for (int i = 0; i < stepCount; i++) {
		//find block position at current step
		Vector3s currentHashBlockPosition = TO_SHORT_FLOOR3(currentCheckPosition_HashBlocks);
		int incrementCount;
		if (i > 0 && (incrementCount = getIncrementCount(currentHashBlockPosition, previousHashBlockPosition)) > 1) {
			if (incrementCount == 2) {
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					if (currentHashBlockPosition.values[iDirection] != previousHashBlockPosition.values[iDirection]) {
						Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
						potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
						if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
						                                       1.0f)) {
							MarkForAllocationAndSetVisibilityTypeIfNotFound(
									hashEntryStates,
									blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
									collisionDetected);
						}
					}
				}
			} else {
				//incrementCount == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hashEntryStates,
								blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
								collisionDetected);
					}
					potentiallyMissedBlockPosition = currentHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = previousHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(marchSegment, TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound(
								hashEntryStates,
								blockCoords, blockVisibilityTypes, potentiallyMissedBlockPosition, hashTable,
								collisionDetected);
					}
				}
			}
		}
		MarkForAllocationAndSetVisibilityTypeIfNotFound(hashEntryStates,
		                                                blockCoords,
		                                                blockVisibilityTypes, currentHashBlockPosition, hashTable,
		                                                collisionDetected);

		currentCheckPosition_HashBlocks += strideVector;
		previousHashBlockPosition = currentHashBlockPosition;
	}
}




_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hashEntryPosition, ITMHashEntry* hashTable, ITMHashEntry*& resultEntry,
                             int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                             const int* excessAllocationList, int& hashCode) {
	hashCode = HashCodeFromBlockPosition(hashEntryPosition);
	ITMHashEntry hashEntry = hashTable[hashCode];
	if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hashEntry.ptr >= -1) {
			while (hashEntry.offset >= 1) {
				hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
				hashEntry = hashTable[hashCode];
				if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
					resultEntry = &hashTable[hashCode];
					return true;
				}
			}
			isExcess = true;

		}
		//still not found, allocate
		if (isExcess && lastFreeVoxelBlockId >= 0 && lastFreeExcessListId >= 0) {
			//there is room in the voxel block array and excess list
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			int exlOffset = excessAllocationList[lastFreeExcessListId];
			hashTable[hashCode].offset = exlOffset + 1; //connect to child
			hashCode = ORDERED_LIST_SIZE + exlOffset;
			hashTable[hashCode] = newHashEntry; //add child to the excess list
			resultEntry = &hashTable[hashCode];
			lastFreeVoxelBlockId--;
			lastFreeExcessListId--;
			return true;
		} else if (lastFreeVoxelBlockId >= 0) {
			//there is room in the voxel block array
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			hashTable[hashCode] = newHashEntry;
			resultEntry = &hashTable[hashCode];
			lastFreeVoxelBlockId--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		resultEntry = &hashTable[hashCode];
		return true;
	}
}
