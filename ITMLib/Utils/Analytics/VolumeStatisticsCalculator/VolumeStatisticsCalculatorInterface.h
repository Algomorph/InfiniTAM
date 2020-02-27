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
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../WarpType.h"
#include "../../VoxelFlags.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class VolumeStatisticsCalculatorInterface {
public:
	virtual Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual Vector6i ComputeAlteredVoxelBounds(VoxelVolume<TVoxel, TIndex>* volume) = 0;

	virtual int ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<int> GetAllocatedHashCodes(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<Vector3s> GetAllocatedHashBlockPositions(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual std::vector<int> GetUtilizedHashCodes(VoxelVolume <TVoxel, TIndex>* volume) = 0;
	virtual std::vector<Vector3s> GetUtilizedHashBlockPositions(VoxelVolume <TVoxel, TIndex>* volume) = 0;

	virtual unsigned int ComputeVoxelWithFlagsCount(VoxelVolume<TVoxel, TIndex>* volume, VoxelFlags flags) = 0;
	virtual unsigned int ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual unsigned int CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* volume, float value) = 0;
	virtual double ComputeNonTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* volume) = 0;
	virtual double ComputeTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* volume) = 0;

	virtual double ComputeWarpUpdateMin(VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeWarpUpdateMax(VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeWarpUpdateMean(VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual void ComputeWarpUpdateMaxAndPosition(float& value, Vector3i& position, const VoxelVolume<TVoxel,TIndex>* volume) = 0;
	
	virtual double ComputeFramewiseWarpMin(VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeFramewiseWarpMax(VoxelVolume<TVoxel,TIndex>* volume) = 0;
	virtual double ComputeFramewiseWarpMean(VoxelVolume<TVoxel,TIndex>* volume) = 0;

	virtual Extent3D FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* volume) = 0;
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeStatisticsCalculator;

}//end namespace ITMLib


