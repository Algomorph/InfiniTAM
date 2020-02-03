//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include <iostream>
#include "DynamicCameraTracker.h"

using namespace ITMLib;

DynamicCameraTracker::DynamicCameraTracker(const Vector2i& imgSize_d, const Vector2i& imgSize_rgb, bool useDepth,
                                           bool useColour, float colourWeight, TrackerIterationType* trackingRegime,
                                           int noHierarchyLevels, float terminationThreshold, float failureDetectorThreshold,
                                           float viewFrustum_min, float viewFrustum_max, float minColourGradient,
                                           float tukeyCutOff, int framesToSkip, int framesToWeight,
                                           const LowLevelEngine* lowLevelEngine, MemoryDeviceType memoryType)
		: ExtendedTracker(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, trackingRegime,
		                  noHierarchyLevels, terminationThreshold, failureDetectorThreshold, viewFrustum_min,
		                  viewFrustum_max, minColourGradient, tukeyCutOff, framesToSkip, framesToWeight,
		                  lowLevelEngine, memoryType) {}

void ITMLib::DynamicCameraTracker::TrackCamera(ITMLib::CameraTrackingState* trackingState, const ITMLib::ITMView* view) {
	//obtain initial camera alignment
	ExtendedTracker::TrackCamera(trackingState, view);
	switch (trackingState->trackerResult){
		case CameraTrackingState::TrackingResult::TRACKING_FAILED:
			std::cout << "Tracking failed" << std::endl;
			std::cout << *(trackingState->pose_d) << std::endl;
			break;
		case CameraTrackingState::TrackingResult::TRACKING_GOOD:
			std::cout << "Tracking good" << std::endl;
			std::cout << *(trackingState->pose_d) << std::endl;
			break;
		case CameraTrackingState::TrackingResult::TRACKING_POOR:
			std::cout << "Tracking poor" << std::endl;
			std::cout << *(trackingState->pose_d) << std::endl;
			break;
	}

}

