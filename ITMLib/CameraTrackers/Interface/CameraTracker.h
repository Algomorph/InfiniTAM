// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Tracking/CameraTrackingState.h"
#include "../../Objects/Views/View.h"

namespace ITMLib
{
	/** \brief
	    Basic interface to any sort of trackers that will align an
	    incoming view with an existing scene.
	*/
	class CameraTracker
	{
	public:
		/** Gets whether the tracker can keep tracking or not.
		    Can be used to signal e.g. the end of a sequence
		    of file-based poses, or the failure of an IMU.
		*/
		virtual bool CanKeepTracking() const { return true; }

		/** Localize a View in the given scene. The result is
		    currently stored as an attribute in trackingState.
		*/
		virtual void TrackCamera(CameraTrackingState *trackingState, const View *view) = 0;

		/** Updates the initial pose of the depth camera in the scene.
		    This can be used to make the scene up vector correspond
		    to the real world's up direction.
		*/
		virtual void UpdateInitialPose(CameraTrackingState *trackingState) {}

		virtual bool RequiresColorRendering() const = 0;
		virtual bool RequiresDepthReliability() const = 0;
		virtual bool RequiresPointCloudRendering() const = 0;

		virtual ~CameraTracker() {}
	};
}
