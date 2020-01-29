//  ================================================================
//  Created by Gregory Kramida on 7/12/18.
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
#pragma once

#include "ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../../Objects/Scene/ITMTrilinearInterpolation.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Configuration.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../Common/ITMWarpEnums.h"
#include "../../Common/ITMCommonFunctors.h"
#include "../../VolumeEditAndCopy/Shared/VolumeEditAndCopyEngine_Shared.h"




template<typename TVoxel>
struct TSDFFusionFunctor {
	TSDFFusionFunctor(int maximumWeight) :
			maximumWeight(maximumWeight) {}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel) {
		fuseLiveVoxelIntoCanonical(liveVoxel, maximumWeight, canonicalVoxel);
	}

private:
	const int maximumWeight;
};

