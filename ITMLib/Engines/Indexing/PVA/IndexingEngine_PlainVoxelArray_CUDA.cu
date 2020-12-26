//  ================================================================
//  Created by Gregory Kramida on 11/15/19.
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

#include "IndexingEngine_PlainVoxelArray.tpp"
#include "../../../GlobalTemplateDefines.h"

namespace ITMLib {
template
class IndexingEngine<TSDFVoxel_f_flags, PlainVoxelArray, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_flags, PlainVoxelArray, MEMORYDEVICE_CUDA, DIAGNOSTIC>;
template
class IndexingEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA, DIAGNOSTIC>;
template
class IndexingEngine<TSDFVoxel_f_rgb, PlainVoxelArray, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_rgb, PlainVoxelArray, MEMORYDEVICE_CUDA, DIAGNOSTIC>;
} //namespace ITMLib