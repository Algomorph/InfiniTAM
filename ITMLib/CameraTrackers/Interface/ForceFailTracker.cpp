// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ForceFailTracker.h"

namespace ITMLib
{

//#################### PUBLIC MEMBER FUNCTIONS ####################

void ForceFailTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	trackingState->trackerResult = ITMTrackingState::TRACKING_FAILED;
}

bool ForceFailTracker::requiresColourRendering() const
{
	return false;
}

bool ForceFailTracker::requiresDepthReliability() const
{
	return false;
}

bool ForceFailTracker::requiresPointCloudRendering() const
{
	return false;
}

}
