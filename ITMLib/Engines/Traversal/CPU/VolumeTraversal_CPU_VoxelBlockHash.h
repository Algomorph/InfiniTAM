//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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

//local
#include "../Interface/VolumeTraversal.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../Shared/VolumeTraversal_Shared.h"
#include "../../../Utils/Analytics/IsAltered.h"

namespace ITMLib {


//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR VOLUMES USING VoxelBlockHash FOR INDEXING
//======================================================================================================================

//static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class VolumeTraversalEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> {
public:
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

	template<typename TFunctor>
	inline static void
	VoxelTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		TVoxel* const voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* const hashTable = scene->index.GetEntries();
		const int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	VoxelTraversal_SingleThreaded(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	VoxelPositionTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		TVoxel* const voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* const hashTable = scene->index.GetEntries();
		const int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel, voxelPosition);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	VoxelAndHashBlockPositionTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor) {
		TVoxel* const voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* const hashTable = scene->index.GetEntries();
		const int hashEntryCount = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor)
#endif
		for (int entryId = 0; entryId < hashEntryCount; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel, voxelPosition, currentHashEntry.pos);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	VoxelTraversalWithinBounds(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor,
	                           const Vector6i& bounds) {
		TVoxel* const voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* const hashTable = scene->index.GetEntries();
		const int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(functor, bounds)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}
			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	VoxelPositionTraversalWithinBounds(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor,
	                                   Vector6i bounds) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}
			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel, voxelPosition);
					}
				}
			}
		}
	}

	template<typename TFunctor>
	inline static void
	VoxelPositionAndHashEntryTraversalWithinBounds(VoxelVolume<TVoxel, VoxelBlockHash>* scene, TFunctor& functor,
	                                               Vector6i bounds) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(VOXEL_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}
			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

			functor.processHashEntry(currentHashEntry);

			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
						TVoxel& voxel = localVoxelBlock[locId];
						functor(voxel, voxelPosition);
					}
				}
			}
		}
	}

// endregion ===========================================================================================================
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = localVoxelBlock[locId];
						TStaticFunctor::run(voxel);
					}
				}
			}
		}
	}

	template<typename TStaticFunctor>
	inline static void StaticVoxelPositionTraversal(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const HashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);
			//position of the current entry in 3D space (in voxel units)
			Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) {
				for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) {
					for (int x = 0; x < VOXEL_BLOCK_SIZE; x++) {
						Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
						int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
						TVoxel& voxel = localVoxelBlock[locId];
						TStaticFunctor::run(voxel, voxelPosition);
					}
				}
			}
		}
	}
// endregion
};

}//namespace ITMLib