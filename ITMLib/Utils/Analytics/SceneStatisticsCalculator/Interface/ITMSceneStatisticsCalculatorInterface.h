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
#include "../../../../Objects/Scene/VoxelVolume.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculatorInterface {
public:
	virtual Vector6i ComputeVoxelBounds(const VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual int ComputeAllocatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual std::vector<int> GetFilledHashBlockIds(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual int ComputeAllocatedHashBlockCount(VoxelVolume<TVoxel, TIndex>* scene) = 0;

	virtual int ComputeNonTruncatedVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual unsigned int ComputeAlteredVoxelCount(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual unsigned int CountVoxelsWithSpecificSdfValue(VoxelVolume<TVoxel, TIndex>* scene, float value) = 0;
	virtual double ComputeNonTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual double ComputeTruncatedVoxelAbsSdfSum(VoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual double ComputeFramewiseWarpMin(VoxelVolume<TVoxel,TIndex>* scene) = 0;
	virtual double ComputeFramewiseWarpMax(VoxelVolume<TVoxel,TIndex>* scene) = 0;
	virtual double ComputeFramewiseWarpMean(VoxelVolume<TVoxel,TIndex>* scene) = 0;

	virtual Extent3D FindMinimumNonTruncatedBoundingBox(VoxelVolume <TVoxel, TIndex>* scene) = 0;

	virtual float FindMaxGradient0LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) = 0;
	virtual float FindMaxGradient1LengthAndPosition(VoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) = 0;
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class ITMSceneStatisticsCalculator;

}//end namespace ITMLib


