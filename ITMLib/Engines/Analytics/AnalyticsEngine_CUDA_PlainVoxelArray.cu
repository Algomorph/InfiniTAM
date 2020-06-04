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
#include "../../GlobalTemplateDefines.h"
#include "../Reduction/CUDA/VolumeReduction_CUDA_PlainVoxelArray.h"
#include "../Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h"
#include "../Traversal/CUDA/HashTableTraversal_CUDA.cuh"
#include "AnalyticsEngine.tpp"

namespace ITMLib {
template
class AnalyticsEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;
template
class AnalyticsEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;
} // namespace ITMLib