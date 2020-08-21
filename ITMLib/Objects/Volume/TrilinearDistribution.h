//  ================================================================
//  Created by Gregory Kramida on 2/23/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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


#include "VoxelBlockHash.h"
#include "RepresentationAccess.h"
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/Enums/VoxelFlags.h"

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel&
ReadVoxelRef(THREADPTR(TVoxel)* voxelData, const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* voxelIndex,
             const THREADPTR(Vector3i)& point, THREADPTR(int)& vmIndex,
             THREADPTR(ITMLib::VoxelBlockHash::IndexCache)& cache) {
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos) {
		vmIndex = true;
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = HashCodeFromBlockPosition(blockPos);

	while (true) {
		HashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
			cache.blockPos = blockPos;
			cache.blockPtr = hashEntry.ptr * VOXEL_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound

			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}
	static TVoxel emptyVoxel;

	vmIndex = false;
	return emptyVoxel;
}

//sdf only
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void DistributeTrilinearly(THREADPTR(TVoxel)* voxels,
                                  const CONSTPTR(HashEntry)* hashEntries,
                                  THREADPTR(TCache)& cache,
                                  const CONSTPTR(Vector3f)& point,
                                  const CONSTPTR(float)& sdfValue) {
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
	Vector3f invCoeff = Vector3f(1.f) - coeff;

	float f0 = invCoeff.z * sdfValue;
	float f1 = coeff.z * sdfValue;

	float f00 = invCoeff.y * f0;
	float f01 = coeff.y * f0;
	float f10 = invCoeff.y * f1;
	float f11 = coeff.y * f1;

	TVoxel& v000 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(0, 0, 0), vmIndex, cache);
	TVoxel& v100 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(1, 0, 0), vmIndex, cache);
	TVoxel& v010 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(0, 1, 0), vmIndex, cache);
	TVoxel& v110 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(1, 1, 0), vmIndex, cache);
	TVoxel& v001 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(0, 0, 1), vmIndex, cache);
	TVoxel& v101 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(1, 0, 1), vmIndex, cache);
	TVoxel& v011 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(0, 1, 1), vmIndex, cache);
	TVoxel& v111 = ReadVoxelRef(voxels, hashEntries, pos + Vector3i(1, 1, 1), vmIndex, cache);

	v000.sdf = invCoeff.x * f00;
	v100.sdf = coeff.x * f00;
	v001.sdf = invCoeff.x * f01;
	v101.sdf = coeff.x * f01;
	v010.sdf = invCoeff.x * f10;
	v110.sdf = coeff.x * f10;
	v011.sdf = invCoeff.x * f11;
	v111.sdf = coeff.x * f11;
}


