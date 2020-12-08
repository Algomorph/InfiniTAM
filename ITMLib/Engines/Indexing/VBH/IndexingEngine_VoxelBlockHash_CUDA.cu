//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

//local
#include "../../../GlobalTemplateDefines.h"
#include "../../Traversal/CUDA/ImageTraversal_CUDA.cuh"
#include "../../Traversal/CUDA/TwoImageTraversal_CUDA.h"
#include "../../Traversal/CUDA/RawArrayTraversal_CUDA.cuh"
#include "../../Traversal/CUDA/MemoryBlockTraversal_CUDA.cuh"
#include "../../Traversal/CUDA/HashTableTraversal_CUDA.cuh"
#include "../../Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#include "IndexingEngine_VoxelBlockHash.tpp"
#include "CUDA/IndexingEngine_VoxelBlockHash_CUDA.tcu"


namespace ITMLib {

template
class IndexingEngine<TSDFVoxel_f_flags, VoxelBlockHash, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_flags, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>;
template
class IndexingEngine<TSDFVoxel_f_rgb, VoxelBlockHash, MEMORYDEVICE_CUDA, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_rgb, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>;

}//namespace ITMLib