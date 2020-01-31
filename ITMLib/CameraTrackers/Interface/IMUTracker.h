// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"
#include "../../Objects/Misc/ITMIMUMeasurement.h"

namespace ITMLib
{
	class IMUTracker : public CameraTracker
	{
	private:
		ITMIMUCalibrator *calibrator;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);
		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return false; }

		IMUTracker(ITMIMUCalibrator *calibrator);
		virtual ~IMUTracker(void);
	};
}
