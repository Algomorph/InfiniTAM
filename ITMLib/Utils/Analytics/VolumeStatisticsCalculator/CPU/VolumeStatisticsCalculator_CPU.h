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
#pragma once

#include <vector>
#include "../../../Math.h"
#include "../../../../Objects/Volume/VoxelVolume.h"
#include "../Interface/VolumeStatisticsCalculatorInterface.h"
#include "../../../../GlobalTemplateDefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class VolumeStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU> :
		public VolumeStatisticsCalculatorInterface<TVoxel, TIndex> {
public:
	static VolumeStatisticsCalculator& Instance() {
		static VolumeStatisticsCalculator instance;
		return instance;
	}

	VolumeStatisticsCalculator(VolumeStatisticsCalculator const&) = delete;
	void operator=(VolumeStatisticsCalculator const&) = delete;

	Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) override;
	Vector6i ComputeAlteredVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) override;
	int ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) override;
	int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<int> GetAllocatedHashCodes(VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetAllocatedHashBlockPositions(VoxelVolume<TVoxel, TIndex>* volume) override;

	int ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* volume, float value) override;
	double ComputeNonTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* volume) override;
	double ComputeTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* volume) override;
	double ComputeFramewiseWarpMin(VoxelVolume<TVoxel,TIndex>* volume) override;
	double ComputeFramewiseWarpMax(VoxelVolume<TVoxel,TIndex>* volume) override;
	double ComputeFramewiseWarpMean(VoxelVolume<TVoxel,TIndex>* volume) override;

	Vector6i FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* volume) override;

	float FindMaxGradient0LengthAndPosition(VoxelVolume<TVoxel, TIndex>* volume, Vector3i& positionOut) override;
	float FindMaxGradient1LengthAndPosition(VoxelVolume<TVoxel, TIndex>* volume, Vector3i& positionOut) override;
private:
	VolumeStatisticsCalculator() = default;
	~VolumeStatisticsCalculator() = default;
};

typedef VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> StatCalc_CPU_VBH_Voxel;
typedef VolumeStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> StatCalc_CPU_PVA_Voxel;
typedef VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> StatCalc_CPU_VBH_Warp;
typedef VolumeStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> StatCalc_CPU_PVA_Warp;

}//end namespace ITMLib

