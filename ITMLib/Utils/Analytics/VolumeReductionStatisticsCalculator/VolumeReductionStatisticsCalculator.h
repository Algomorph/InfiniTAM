//  ================================================================
//  Created by Gregory Kramida on 2/21/20.
//  Copyright (c) 2020 Gregory Kramida
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

#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../Statistics.h"
#include "../../Math.h"
#include "../../../Objects/Volume/VoxelVolume.h"

namespace ITMLib{

template <typename TVoxel, typename TIndex>
class VolumeReductionStatisticsCalculatorInterface{
	virtual void ComputeWarpUpdateLengthMax(float& max, Vector3i& position, VoxelVolume<TVoxel,TIndex>* volume) = 0;
};

template  <typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class VolumeReductionStatisticsCalculator;

} // namespace ITMLib