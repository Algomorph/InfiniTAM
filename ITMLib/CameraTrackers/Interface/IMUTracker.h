// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/Preprocessing/Interface/PreprocessingEngineInterface.h"
#include "../../Objects/Misc/IMUCalibrator.h"
#include "../../Objects/Misc/IMUMeasurement.h"

namespace ITMLib
{
	class IMUTracker : public CameraTracker
	{
	private:
		IMUCalibrator *calibrator;

	public:
		void TrackCamera(CameraTrackingState *trackingState, const View *view);
		bool requiresColourRendering() const { return false; }
		bool requiresDepthReliability() const { return false; }
		bool requiresPointCloudRendering() const { return false; }

		IMUTracker(IMUCalibrator *calibrator);
		virtual ~IMUTracker();
	};
}
