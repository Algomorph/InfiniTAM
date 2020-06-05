//  ================================================================
//  Created by Gregory Kramida on 10/10/19.
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

//============================= PARAMETER/SWITCH DATA STRUCTURES FOR PRECONFIGURED OPTIMIZATION SETTINGS ===============
namespace ITMLib{
	struct SurfaceTrackerOptimizationParameters {
		const float learning_rate;// = 0.1f;
		const float rigidity_enforcement_factor;// = 0.1f;
		const float weight_data_term;
		const float weight_smoothing_term;// = 0.2f; //0.2 is default for SobolevFusion, 0.5 is default for KillingFusion
		const float weight_level_set_term;// = 0.2f;
		const float epsilon;// = 1e-5f;
		const float unity; // voxel_size / truncation_distance, i.e. 1 / [narrow-band half-width in voxels] or [voxel size in metric units] / [narrow-band half-width in metric units]
	};

	struct ITMSceneMotionOptimizationSwitches {
		const bool enable_data_term;
		const bool enable_level_set_term;
		const bool enableTikhonovTerm;
		const bool enable_killing_rigidity_enforcement_term;
		const bool enable_sobolev_gradient_smoothing;
	};
} // namespace ITMLib


