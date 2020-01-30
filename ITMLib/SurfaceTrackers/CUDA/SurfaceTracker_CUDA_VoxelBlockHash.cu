//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include "../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#include "../../Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../../Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#include "../../Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#include "../WarpGradientFunctors/WarpGradientFunctor_SlavchevaOptimized.h"
#include "../WarpGradientFunctors/WarpGradientFunctor_SlavchevaDiagnostic.h"
#include "../../ITMLibDefines.h"
#include "../Interface/SurfaceTracker.tpp"

namespace ITMLib {
template
class SurfaceTracker<ITMVoxel, ITMWarp, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_OPTIMIZED>;
template
class SurfaceTracker<ITMVoxel, ITMWarp, VoxelBlockHash, MEMORYDEVICE_CUDA, TRACKER_SLAVCHEVA_DIAGNOSTIC>;
} // namespace ITMLib