//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
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

#include <vector>
#include "../../Math.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "VolumeStatisticsCalculatorInterface.h"
#include "../../../GlobalTemplateDefines.h"

namespace ITMLib {



template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeStatisticsCalculator :
		public VolumeStatisticsCalculatorInterface<TVoxel, TIndex> {
public:
	static VolumeStatisticsCalculator& Instance() {
		static VolumeStatisticsCalculator instance;
		return instance;
	}

	VolumeStatisticsCalculator(VolumeStatisticsCalculator const&) = delete;
	void operator=(VolumeStatisticsCalculator const&) = delete;

	Vector6i ComputeVoxelBounds(const VoxelVolume <TVoxel, TIndex>* volume) override;
	Vector6i ComputeAlteredVoxelBounds(VoxelVolume <TVoxel, TIndex>* volume) override;

	unsigned int CountAllocatedVoxels(VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountUtilizedVoxels(VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountHashBlocksWithDepthWeightInRange(VoxelVolume <TVoxel, TIndex>* volume, Extent2Di range) override;
	unsigned int CountVoxelsWithDepthWeightInRange(VoxelVolume <TVoxel, TIndex>* volume, Extent2Di range) override;
	unsigned int CountVoxelsWithSpecifiedFlags(VoxelVolume <TVoxel, TIndex>* volume, VoxelFlags flags) override;
	unsigned int CountNonTruncatedVoxels(VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountAlteredVoxels(VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountVoxelsWithSpecificSdfValue(VoxelVolume <TVoxel, TIndex>* volume, float value) override;
	double SumNonTruncatedVoxelAbsSdf(VoxelVolume <TVoxel, TIndex>* volume) override;
	double SumTruncatedVoxelAbsSdf(VoxelVolume <TVoxel, TIndex>* volume) override;

	double ComputeWarpUpdateMin(VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeWarpUpdateMax(VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeWarpUpdateMean(VoxelVolume <TVoxel, TIndex>* volume) override;

	double ComputeFramewiseWarpMin(VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeFramewiseWarpMax(VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeFramewiseWarpMean(VoxelVolume <TVoxel, TIndex>* volume) override;

	void ComputeWarpUpdateMaxAndPosition(float& value, Vector3i& position, const VoxelVolume<TVoxel,TIndex>* volume);



	Vector6i FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* volume) override;


	unsigned int CountAllocatedHashBlocks(VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountUtilizedHashBlocks(VoxelVolume <TVoxel, TIndex>* volume) override;
	std::vector<int> GetAllocatedHashCodes(VoxelVolume <TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetAllocatedHashBlockPositions(VoxelVolume <TVoxel, TIndex>* volume) override;
	std::vector<int> GetUtilizedHashCodes(VoxelVolume <TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetUtilizedHashBlockPositions(VoxelVolume <TVoxel, TIndex>* volume) override;

private:
	VolumeStatisticsCalculator() = default;
	~VolumeStatisticsCalculator() = default;
};

extern template
class VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
extern template
class VolumeStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;
extern template
class VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
extern template
class VolumeStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;
extern template
class VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template
class VolumeStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;
extern template
class VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template
class VolumeStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;

typedef VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> StatCalc_CPU_VBH_Voxel;
typedef VolumeStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> StatCalc_CPU_PVA_Voxel;
typedef VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> StatCalc_CPU_VBH_Warp;
typedef VolumeStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> StatCalc_CPU_PVA_Warp;
typedef VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> StatCalc_CUDA_VBH_Voxel;
typedef VolumeStatisticsCalculator<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> StatCalc_CUDA_PVA_Voxel;
typedef VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> StatCalc_CUDA_VBH_Warp;
typedef VolumeStatisticsCalculator<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> StatCalc_CUDA_PVA_Warp;

}//end namespace ITMLib

