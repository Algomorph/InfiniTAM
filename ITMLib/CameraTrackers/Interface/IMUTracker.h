// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/LowLevel/Interface/LowLevelEngine.h"
#include "../../Objects/Misc/IMUCalibrator.h"
#include "../../Objects/Misc/IMUMeasurement.h"

namespace ITMLib
{
	class IMUTracker : public CameraTracker
	{
	private:
		IMUCalibrator *calibrator;

	public:
		void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);
		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return false; }

		IMUTracker(IMUCalibrator *calibrator);
		virtual ~IMUTracker(void);
	};
}
