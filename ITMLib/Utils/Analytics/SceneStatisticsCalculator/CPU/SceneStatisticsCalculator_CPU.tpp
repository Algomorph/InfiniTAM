//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
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
//local
#include "SceneStatisticsCalculator_CPU.h"
#include "../../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/VoxelTypes.h"
#include "../Shared/SceneStatisticsCalculator_Functors.h"

//atomic
#include <atomic>

using namespace ITMLib;


template<typename TVoxel, typename TIndex>
struct ComputeVoxelBoundsFunctor;

template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, VoxelBlockHash> {
	static Vector6i Compute(const VoxelVolume<TVoxel, VoxelBlockHash>* scene) {

		Vector6i bounds = Vector6i(0);

		const TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;

		//TODO: if OpenMP standard is 3.1 or above, use OpenMP parallel for reduction clause with (max:maxVoxelPointX,...) -Greg (GitHub: Algomorph)
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {

			const ITMHashEntry& currentHashEntry = hashTable[entryId];

			if (currentHashEntry.ptr < 0) continue;

			//position of the current entry in 3D space
			Vector3i currentHashBlockPositionVoxels = currentHashEntry.pos.toInt() * VOXEL_BLOCK_SIZE;
			Vector3i hashBlockLimitPositionVoxels =
					(currentHashEntry.pos.toInt() + Vector3i(1, 1, 1)) * VOXEL_BLOCK_SIZE;

			if (bounds.min_x > currentHashBlockPositionVoxels.x) {
				bounds.min_x = currentHashBlockPositionVoxels.x;
			}
			if (bounds.max_x < hashBlockLimitPositionVoxels.x) {
				bounds.max_x = hashBlockLimitPositionVoxels.x;
			}
			if (bounds.min_y > currentHashBlockPositionVoxels.y) {
				bounds.min_y = currentHashBlockPositionVoxels.y;
			}
			if (bounds.max_y < hashBlockLimitPositionVoxels.y) {
				bounds.max_y = hashBlockLimitPositionVoxels.y;
			}
			if (bounds.min_z > currentHashBlockPositionVoxels.z) {
				bounds.min_z = currentHashBlockPositionVoxels.z;
			}
			if (bounds.max_z < hashBlockLimitPositionVoxels.z) {
				bounds.max_z = hashBlockLimitPositionVoxels.z;
			}
		}
		return bounds;
	}
};

template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, PlainVoxelArray> {
	static Vector6i Compute(const VoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		Vector3i offset = scene->index.GetVolumeOffset();
		Vector3i size = scene->index.GetVolumeSize();
		return {offset.x, offset.y, offset.z,
		        offset.x + size.x, offset.y + size.y, offset.z + size.z};
	}
};

template<typename TVoxel, typename TIndex>
Vector6i
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeVoxelBoundsFunctor<TVoxel, TIndex>::Compute(scene);
}

//============================================== COUNT VOXELS ==========================================================
template<typename TVoxel, typename TIndex>
struct ComputeAllocatedVoxelCountFunctor;

template<typename TVoxel>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, VoxelBlockHash> {
	static int Compute(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		int count = 0;

		const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:count)
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			count += VOXEL_BLOCK_SIZE3;
		}
		return count;
	}
};

template<typename TVoxel>
struct ComputeAllocatedVoxelCountFunctor<TVoxel, PlainVoxelArray> {
	static int Compute(VoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		return scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y * scene->index.GetVolumeSize().z;
	}
};

template<typename TVoxel, typename TIndex>
int
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeAllocatedVoxelCountFunctor<TVoxel, TIndex>::Compute(scene);
}

template<bool hasSemanticInformation, typename TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor;

template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<false, TVoxel, TIndex> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION("Voxels need to have semantic information to be marked as truncated or non-truncated.");
	}
};
template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<true, TVoxel, TIndex> {
	static int compute(VoxelVolume<TVoxel, TIndex>* scene) {
		ComputeNonTruncatedVoxelCountFunctor instance;
		VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::VoxelTraversal_SingleThreaded(scene,
		                                                                                       instance);
		return instance.count;
	}

	int count = 0;

	void operator()(TVoxel& voxel) {
		count += voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
	}
};


template<typename TVoxel, typename TIndex>
int
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeNonTruncatedVoxelCountFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex>::compute(scene);
}


template<typename TVoxel, typename TIndex>
double
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeNonTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* scene) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::compute(
			scene, VoxelFlags::VOXEL_NONTRUNCATED);

}

template<typename TVoxel, typename TIndex>
double
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* scene) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::
	compute(scene, VoxelFlags::VOXEL_TRUNCATED);
}

//======================================================================================================================




template<typename TVoxel, typename TIndex>
struct HashOnlyStatisticsFunctor;
template<typename TVoxel>
struct HashOnlyStatisticsFunctor<TVoxel, PlainVoxelArray> {
	static std::vector<int> GetFilledHashBlockIds(VoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		return std::vector<int>();
	}

	static int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, PlainVoxelArray>* scene) {
		return 0;
	}
};
template<typename TVoxel>
struct HashOnlyStatisticsFunctor<TVoxel, VoxelBlockHash> {
	static std::vector<int> GetFilledHashBlockIds(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		std::vector<int> ids;
		const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			ids.push_back(entryId);
		}
		return ids;
	}

	static int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, VoxelBlockHash>* scene) {
		int count = 0;
		const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.hashEntryCount;
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
			if (currentHashEntry.ptr >= 0) count++;
		}
		return count;
	}
};

template<typename TVoxel, typename TIndex>
std::vector<int>
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::GetFilledHashBlockIds(VoxelVolume<TVoxel, TIndex>* scene) {
	HashOnlyStatisticsFunctor<TVoxel, TIndex>::GetFilledHashBlockIds(scene);
	DIEWITHEXCEPTION("Not implemented");
}

template<typename TVoxel, typename TIndex>
int
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAllocatedHashBlockCount(
		VoxelVolume<TVoxel, TIndex>* scene) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex>::ComputeAllocatedHashBlockCount(scene);
}


template<typename TVoxel, typename TIndex>
unsigned int
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* scene,
                                                                                                float value) {
	return ComputeVoxelCountWithSpecificValue<TVoxel::hasSDFInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::compute(
			scene, value);
}

// region ================================ VOXEL GRADIENTS =============================================================

template<typename TVoxel, typename TIndex>
struct MaxGradientFunctor;

template<typename TIndex>
struct MaxGradientFunctor<TSDFVoxel_f_flags, TIndex> {
	static float
	find(VoxelVolume<TSDFVoxel_f_flags, TIndex>* scene, bool secondGradientField, Vector3i& maxPosition) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not implemented for voxels without gradients.");
	}
};

template<typename TIndex>
struct MaxGradientFunctor<WarpVoxel_f_uf, TIndex> {
	static float find(VoxelVolume<WarpVoxel, TIndex>* scene, bool secondGradientField, Vector3i& maxPosition) {
		MaxGradientFunctor maxGradientFunctor;
		maxGradientFunctor.secondGradientField = secondGradientField;
		VolumeTraversalEngine<WarpVoxel_f_uf, TIndex, MEMORYDEVICE_CPU>::
		VoxelPositionTraversal(scene, maxGradientFunctor);
		maxPosition = maxGradientFunctor.maxPosition;
		return maxGradientFunctor.maxLength;
	}

	float maxLength = 0.0f;
	bool secondGradientField;
	Vector3i maxPosition = Vector3i(0);

	void operator()(WarpVoxel_f_uf& voxel, Vector3i position) {
		float gradientLength;
		if (secondGradientField) {
			gradientLength = ORUtils::length(voxel.gradient1);
		} else {
			gradientLength = ORUtils::length(voxel.gradient0);
		}
		if (gradientLength > maxLength) {
			maxLength = gradientLength;
			maxPosition = position;
		}
	}
};


template<typename TVoxel, typename TIndex>
float
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMaxGradient0LengthAndPosition(
		VoxelVolume<TVoxel, TIndex>* scene,
		Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(scene, false, positionOut);
}

template<typename TVoxel, typename TIndex>
float
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMaxGradient1LengthAndPosition(
		VoxelVolume<TVoxel, TIndex>* scene,
		Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(scene, true, positionOut);
}


template<typename TVoxel, typename TIndex>
unsigned int
ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) {
	IsAlteredCountFunctor<TVoxel> functor;
	VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::VoxelTraversal(scene, functor);
	return functor.GetCount();
}

template<typename TVoxel, typename TIndex>
double ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMin(VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MINIMUM>::compute(
			scene);
}

template<typename TVoxel, typename TIndex>
double ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMax(VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MAXIMUM>::compute(
			scene);
}

template<typename TVoxel, typename TIndex>
double ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMean(VoxelVolume<TVoxel, TIndex>* scene) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MEAN>::compute(
			scene);
}

template<typename TVoxel, typename TIndex>
Vector6i ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMinimumNonTruncatedBoundingBox(
		VoxelVolume<TVoxel, TIndex>* scene) {
	return FlagMatchBBoxFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::
	        compute(scene, VoxelFlags::VOXEL_NONTRUNCATED);
}

// endregion ===========================================================================================================

