//  ================================================================
//  Created by Gregory Kramida on 10/17/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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
#include "DynamicCameraTracker_CUDA.h"


ITMLib::DynamicCameraTracker_CUDA::DynamicCameraTracker_CUDA(const Vector2i& imgSize_d, const Vector2i& imgSize_rgb,
                                                             bool useDepth, bool useColour, float colourWeight,
                                                             ITMLib::TrackerIterationType* trackingRegime,
                                                             int noHierarchyLevels, float terminationThreshold,
                                                             float failureDetectorThreshold, float viewFrustum_min,
                                                             float viewFrustum_max, float minColourGradient, float tukeyCutOff,
                                                             int framesToSkip, int framesToWeight,
                                                             const ITMLib::ImageProcessingEngineInterface* lowLevelEngine)
		: ExtendedTracker(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime,
		                  noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min,
		                  viewFrustum_max, minColourGradient, tukeyCutOff, framesToSkip, framesToWeight,
		                  lowLevelEngine, MEMORYDEVICE_CUDA),
		  DynamicCameraTracker(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime,
		                       noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min,
		                       viewFrustum_max, minColourGradient, tukeyCutOff, framesToSkip, framesToWeight,
		                       lowLevelEngine, MEMORYDEVICE_CUDA),
		  ExtendedTracker_CUDA(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime,
		                       noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min,
		                       viewFrustum_max, minColourGradient, tukeyCutOff, framesToSkip, framesToWeight,
		                       lowLevelEngine) {}
