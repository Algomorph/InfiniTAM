//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/13/21.
//  Copyright (c) 2021 Gregory Kramida
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

// local
#include "../Common/Configurable.h"
#include "RigidAlignmentSettings.h"
#include "../../Objects/Tracking/CameraTrackingState.h"
#include "../../Objects/Views/View.h"

namespace ITMLib {
/**
 * \brief Interface for engines that can align
 * incoming frame data to an existing volume.
 * */
class RigidAlignmentEngineInterface : public Configurable<RigidAlignmentSettings> {
protected:
	using Configurable<RigidAlignmentSettings>::parameters;
public:
	/**
	 * \brief Whether the tracker can keep tracking or not.
	 * Can be used to signal e.g. the end of a sequence
	 * of file-based poses, or the failure of an IMU.
	*/
	virtual bool CanKeepTracking() const { return true; }

	/**
	 * \brief Localize a View in the given scene. The result is
	 * currently stored as an attribute in trackingState.
	*/
	virtual void TrackCamera(CameraTrackingState* trackingState, const View* view) = 0;

	/**
	 * Updates the initial pose of the depth camera in the scene.
	 * This can be used to make the scene up vector correspond
	 * to the real world's up direction.
	*/
	virtual void UpdateInitialPose(CameraTrackingState* trackingState) {}

	virtual bool RequiresColourRendering() const = 0;
	virtual bool RequiresDepthReliability() const = 0;
	virtual bool RequiresPointCloudRendering() const = 0;

	virtual ~CameraTracker() {}
};
} // namespace ITMLib