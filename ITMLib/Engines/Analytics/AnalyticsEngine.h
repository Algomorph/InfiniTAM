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
#include "AnalyticsEngineInterface.h"
#include "../../Utils/Math.h"
#include "../../Utils/Analytics/Histogram.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../GlobalTemplateDefines.h"

namespace ITMLib {

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class AnalyticsEngine :
		public AnalyticsEngineInterface<TVoxel, TIndex> {
public:
	static AnalyticsEngine& Instance() {
		static AnalyticsEngine instance;
		return instance;
	}

	AnalyticsEngine(AnalyticsEngine const&) = delete;
	void operator=(AnalyticsEngine const&) = delete;

	Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) override;
	Vector6i ComputeAlteredVoxelBounds(const VoxelVolume <TVoxel, TIndex>* volume) override;

	unsigned int CountAllocatedVoxels(const VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int CountUtilizedVoxels(const VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int CountHashBlocksWithDepthWeightInRange(const VoxelVolume <TVoxel, TIndex>* volume, Extent2Di range) override;
	unsigned int CountVoxelsWithDepthWeightInRange(const VoxelVolume<TVoxel, TIndex>* volume, Extent2Di range) override;
	unsigned int CountVoxelsWithSpecifiedFlags(const VoxelVolume <TVoxel, TIndex>* volume, VoxelFlags flags) override;
	unsigned int CountNonTruncatedVoxels(const VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountAlteredVoxels(const VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountAlteredGradients(const VoxelVolume <TVoxel, TIndex>* volume) override;
	unsigned int CountAlteredWarpUpdates(const VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int CountVoxelsWithSpecificSdfValue(const VoxelVolume<TVoxel, TIndex>* volume, float value) override;

	double SumNonTruncatedVoxelAbsSdf(const VoxelVolume <TVoxel, TIndex>* volume) override;
	double SumTruncatedVoxelAbsSdf(const VoxelVolume<TVoxel, TIndex>* volume) override;

	double ComputeWarpUpdateMin(const VoxelVolume<TVoxel, TIndex>* volume) override;
	double ComputeWarpUpdateMax(const VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeWarpUpdateMean(const VoxelVolume <TVoxel, TIndex>* volume) override;

	double ComputeFramewiseWarpMin(const VoxelVolume <TVoxel, TIndex>* volume) override;
	double ComputeFramewiseWarpMax(const VoxelVolume<TVoxel, TIndex>* volume) override;
	double ComputeFramewiseWarpMean(const VoxelVolume<TVoxel, TIndex>* volume) override;

	void ComputeWarpUpdateMaxAndPosition(float& value, Vector3i& position, const VoxelVolume<TVoxel, TIndex>* volume) override;

	Vector6i FindMinimumNonTruncatedBoundingBox(const VoxelVolume <TVoxel, TIndex>* volume) override;

	unsigned int CountAllocatedHashBlocks(const VoxelVolume<TVoxel, TIndex>* volume) override;
	unsigned int CountUtilizedHashBlocks(const VoxelVolume<TVoxel, TIndex>* volume) override;

	std::vector<int> GetAllocatedHashCodes(const VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetAllocatedHashBlockPositions(const VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<int> GetUtilizedHashCodes(const VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetUtilizedHashBlockPositions(const VoxelVolume<TVoxel, TIndex>* volume) override;
	std::vector<Vector3s> GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(const VoxelVolume<TVoxel, TIndex>* volume) override;

	Histogram ComputeWarpUpdateLengthHistogram_VolumeMax(const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float& maximum) override;
	Histogram ComputeWarpUpdateLengthHistogram_ManualMax(const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float maximum) override;

private:
	Histogram
	ComputeWarpUpdateLengthHistogram(const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float& maximum, bool use_manual_max);

	AnalyticsEngine() = default;
	~AnalyticsEngine() = default;
};

extern template
class AnalyticsEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
extern template
class AnalyticsEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;
extern template
class AnalyticsEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
extern template
class AnalyticsEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;
extern template
class AnalyticsEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template
class AnalyticsEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;
extern template
class AnalyticsEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
extern template
class AnalyticsEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;

typedef AnalyticsEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> Analytics_CPU_VBH_Voxel;
typedef AnalyticsEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> Analytics_CPU_PVA_Voxel;
typedef AnalyticsEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> Analytics_CPU_VBH_Warp;
typedef AnalyticsEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU> Analytics_CPU_PVA_Warp;
typedef AnalyticsEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> Analytics_CUDA_VBH_Voxel;
typedef AnalyticsEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> Analytics_CUDA_PVA_Voxel;
typedef AnalyticsEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> Analytics_CUDA_VBH_Warp;
typedef AnalyticsEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA> Analytics_CUDA_PVA_Warp;

}//end namespace ITMLib

