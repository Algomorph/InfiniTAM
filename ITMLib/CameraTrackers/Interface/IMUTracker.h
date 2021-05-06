// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CameraTracker.h"
#include "../../Engines/ImageProcessing/ImageProcessingEngineInterface.h"
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
		bool RequiresColorRendering() const { return false; }
		bool RequiresDepthReliability() const { return false; }
		bool RequiresPointCloudRendering() const { return false; }

		IMUTracker(IMUCalibrator *calibrator);
		virtual ~IMUTracker();
	};
}
