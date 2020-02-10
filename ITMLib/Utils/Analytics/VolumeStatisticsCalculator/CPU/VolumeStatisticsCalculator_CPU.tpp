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
#include "VolumeStatisticsCalculator_CPU.h"
#include "../../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/VoxelTypes.h"
#include "../Shared/VolumeStatisticsCalculator_Functors.h"


//atomic
#include <atomic>

using namespace ITMLib;


template<typename TVoxel, typename TIndex>
struct ComputeVoxelBoundsFunctor;

template<typename TVoxel>
struct ComputeVoxelBoundsFunctor<TVoxel, VoxelBlockHash> {
	static Vector6i Compute(const VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

		Vector6i bounds = Vector6i(0);

		const TVoxel* voxelBlocks = volume->localVBA.GetVoxelBlocks();
		const HashEntry* hashTable = volume->index.GetEntries();
		int noTotalEntries = volume->index.hashEntryCount;

		//TODO: if OpenMP standard is 3.1 or above, use OpenMP parallel for reduction clause with (max:maxVoxelPointX,...) -Greg (GitHub: Algomorph)
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {

			const HashEntry& currentHashEntry = hashTable[entryId];

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
	static Vector6i Compute(const VoxelVolume<TVoxel, PlainVoxelArray>* volume) {
		Vector3i offset = volume->index.GetVolumeOffset();
		Vector3i size = volume->index.GetVolumeSize();
		return {offset.x, offset.y, offset.z,
		        offset.x + size.x, offset.y + size.y, offset.z + size.z};
	}
};

template<typename TVoxel, typename TIndex>
Vector6i
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeVoxelBoundsFunctor<TVoxel, TIndex>::Compute(volume);
}

//============================================== COUNT VOXELS ==========================================================
template<typename TVoxel, typename TIndex>
int
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeAllocatedVoxelCountFunctor<TVoxel, TIndex, MEMORYDEVICE_CPU>::compute(volume);
}

template<bool hasSemanticInformation, typename TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor;

template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<false, TVoxel, TIndex> {
	static int compute(VoxelVolume<TVoxel, TIndex>* volume) {
		DIEWITHEXCEPTION("Voxels need to have semantic information to be marked as truncated or non-truncated.");
	}
};
template<class TVoxel, typename TIndex>
struct ComputeNonTruncatedVoxelCountFunctor<true, TVoxel, TIndex> {
	static int compute(VoxelVolume<TVoxel, TIndex>* volume) {
		ComputeNonTruncatedVoxelCountFunctor instance;
		VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::VoxelTraversal_SingleThreaded(volume,
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
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeNonTruncatedVoxelCountFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex>::compute(volume);
}


template<typename TVoxel, typename TIndex>
double
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeNonTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::compute(
			volume, VoxelFlags::VOXEL_NONTRUNCATED);

}

template<typename TVoxel, typename TIndex>
double
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeTruncatedVoxelAbsSdfSum(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return SumSDFFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::
	compute(volume, VoxelFlags::VOXEL_TRUNCATED);
}

//======================================================================================================================





template<typename TVoxel, typename TIndex>
std::vector<int>
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::GetAllocatedHashCodes(VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, MEMORYDEVICE_CPU>::GetAllocatedHashCodes(volume);
}

template<typename TVoxel, typename TIndex>
int
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAllocatedHashBlockCount(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return HashOnlyStatisticsFunctor<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAllocatedHashBlockCount(volume);
}


template<typename TVoxel, typename TIndex>
unsigned int
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* volume,
                                                                                              float value) {
	return ComputeVoxelCountWithSpecificValue<TVoxel::hasSDFInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::compute(
			volume, value);
}

// region ================================ VOXEL GRADIENTS =============================================================

template<typename TVoxel, typename TIndex>
struct MaxGradientFunctor;

template<typename TIndex>
struct MaxGradientFunctor<TSDFVoxel_f_flags, TIndex> {
	static float
	find(VoxelVolume<TSDFVoxel_f_flags, TIndex>* volume, bool secondGradientField, Vector3i& maxPosition) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not implemented for voxels without gradients.");
	}
};

template<typename TIndex>
struct MaxGradientFunctor<WarpVoxel_f_uf, TIndex> {
	static float find(VoxelVolume<WarpVoxel, TIndex>* volume, bool secondGradientField, Vector3i& maxPosition) {
		MaxGradientFunctor maxGradientFunctor;
		maxGradientFunctor.secondGradientField = secondGradientField;
		VolumeTraversalEngine<WarpVoxel_f_uf, TIndex, MEMORYDEVICE_CPU>::
		VoxelPositionTraversal(volume, maxGradientFunctor);
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
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMaxGradient0LengthAndPosition(
		VoxelVolume<TVoxel, TIndex>* volume,
		Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(volume, false, positionOut);
}

template<typename TVoxel, typename TIndex>
float
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMaxGradient1LengthAndPosition(
		VoxelVolume<TVoxel, TIndex>* volume,
		Vector3i& positionOut) {
	return MaxGradientFunctor<TVoxel, TIndex>::find(volume, true, positionOut);
}


template<typename TVoxel, typename TIndex>
unsigned int
VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) {
	IsAlteredCountFunctor<TVoxel> functor;
	VolumeTraversalEngine<TVoxel, TIndex, MEMORYDEVICE_CPU>::VoxelTraversal(volume, functor);
	return functor.GetCount();
}

template<typename TVoxel, typename TIndex>
double VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMin(VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MINIMUM>::compute(
			volume);
}

template<typename TVoxel, typename TIndex>
double VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMax(VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MAXIMUM>::compute(
			volume);
}

template<typename TVoxel, typename TIndex>
double VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::ComputeFramewiseWarpMean(VoxelVolume<TVoxel, TIndex>* volume) {
	return ComputeFramewiseWarpLengthStatisticFunctor<TVoxel::hasFramewiseWarp, TVoxel, TIndex, MEMORYDEVICE_CPU, MEAN>::compute(
			volume);
}

template<typename TVoxel, typename TIndex>
Vector6i VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::FindMinimumNonTruncatedBoundingBox(
		VoxelVolume<TVoxel, TIndex>* volume) {
	return FlagMatchBBoxFunctor<TVoxel::hasSemanticInformation, TVoxel, TIndex, MEMORYDEVICE_CPU>::
	        compute(volume, VoxelFlags::VOXEL_NONTRUNCATED);
}

// endregion ===========================================================================================================

