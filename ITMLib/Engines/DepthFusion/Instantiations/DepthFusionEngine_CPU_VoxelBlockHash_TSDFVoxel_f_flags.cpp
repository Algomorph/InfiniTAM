//  ================================================================
//  Created by Gregory Kramida on 6/19/18.
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

#include "../../Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
#include "../../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../DepthFusionEngine.tpp"
#include "../../../Objects/Volume/VoxelTypes.h"

namespace ITMLib {
template
class DepthFusionEngine<TSDFVoxel_f_flags, VoxelBlockHash, MEMORYDEVICE_CPU>;
}