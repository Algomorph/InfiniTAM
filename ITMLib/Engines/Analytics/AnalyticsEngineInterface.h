//  ================================================================
//  Created by Gregory Kramida on 10/1/19.
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


#include <vector>
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Utils/Enums/WarpType.h"
#include "../../Utils/Enums/VoxelFlags.h"
#include "../../Utils/Analytics/Histogram.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class AnalyticsEngineInterface {
public:
	virtual Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual Vector6i ComputeAlteredVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) = 0;

	virtual unsigned int CountUtilizedVoxels(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountAllocatedVoxels(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountVoxelsWithDepthWeightInRange(const VoxelVolume <TVoxel, TIndex>* volume, Extent2Di range) = 0;
	virtual unsigned int CountHashBlocksWithDepthWeightInRange(const VoxelVolume <TVoxel, TIndex>* volume, Extent2Di range) = 0;
	virtual unsigned int CountVoxelsWithSpecifiedFlags(const VoxelVolume<TVoxel, TIndex>* volume, VoxelFlags flags) = 0;
	virtual unsigned int CountNonTruncatedVoxels(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountAlteredVoxels(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountAlteredGradients(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountAlteredWarpUpdates(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountVoxelsWithSpecificSdfValue(const VoxelVolume<TVoxel, TIndex>* volume, float value) = 0;

	virtual double SumNonTruncatedVoxelAbsSdf(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual double SumTruncatedVoxelAbsSdf(const VoxelVolume<TVoxel, TIndex>* volume) = 0;

	virtual double ComputeWarpUpdateMin(const VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeWarpUpdateMax(const VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeWarpUpdateMean(const VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual double ComputeFramewiseWarpMin(const VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeFramewiseWarpMax(const VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeFramewiseWarpMean(const VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual void ComputeWarpUpdateMaxAndPosition(float& value, Vector3i& position, const VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual Extent3Di FindMinimumNonTruncatedBoundingBox(const VoxelVolume <TVoxel, TIndex>* volume) = 0;

	virtual unsigned int CountAllocatedHashBlocks(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountUtilizedHashBlocks(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<int> GetAllocatedHashCodes(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<Vector3s> GetAllocatedHashBlockPositions(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<int> GetUtilizedHashCodes(const VoxelVolume <TVoxel, TIndex>* volume) = 0;
	virtual std::vector<Vector3s> GetUtilizedHashBlockPositions(const VoxelVolume <TVoxel, TIndex>* volume) = 0;
	virtual std::vector<Vector3s> GetDifferenceBetweenAllocatedAndUtilizedHashBlockPositionSets(const VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual Histogram ComputeWarpUpdateLengthHistogram_VolumeMax(const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float& maximum) = 0;
	virtual Histogram ComputeWarpUpdateLengthHistogram_ManualMax(const VoxelVolume<TVoxel, TIndex>* volume, int bin_count, float maximum) = 0;
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class AnalyticsEngine;

}//end namespace ITMLib


