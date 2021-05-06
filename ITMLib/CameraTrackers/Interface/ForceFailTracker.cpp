// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ForceFailTracker.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ForceFailTracker::TrackCamera(CameraTrackingState *trackingState, const View *view)
{
	trackingState->trackerResult = CameraTrackingState::TRACKING_FAILED;
}

bool ForceFailTracker::RequiresColorRendering() const
{
	return false;
}

bool ForceFailTracker::RequiresDepthReliability() const
{
	return false;
}

bool ForceFailTracker::RequiresPointCloudRendering() const
{
	return false;
}

}
