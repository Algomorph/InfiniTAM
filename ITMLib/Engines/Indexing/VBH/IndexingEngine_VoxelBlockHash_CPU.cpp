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
#include "../../Traversal/CPU/ImageTraversal_CPU.h"
#include "../../Traversal/CPU/TwoImageTraversal_CPU.h"
#include "../../Traversal/CPU/RawArrayTraversal_CPU.h"
#include "../../Traversal/CPU/MemoryBlockTraversal_CPU.h"
#include "../../Traversal/CPU/HashTableTraversal_CPU.h"
#include "../../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "IndexingEngine_VoxelBlockHash.tpp"
#include "CPU/IndexingEngine_VoxelBlockHash_CPU.tpp"


namespace ITMLib {

template
class IndexingEngine<TSDFVoxel_f_flags, VoxelBlockHash, MEMORYDEVICE_CPU, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_flags, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, OPTIMIZED>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>;
template
class IndexingEngine<TSDFVoxel_f_rgb, VoxelBlockHash, MEMORYDEVICE_CPU, OPTIMIZED>;
template
class IndexingEngine<TSDFVoxel_f_rgb, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>;

}//namespace ITMLib