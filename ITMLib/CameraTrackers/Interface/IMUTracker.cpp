// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "IMUTracker.h"

#include "../../Objects/Views/ViewIMU.h"
using namespace ITMLib;

IMUTracker::IMUTracker(IMUCalibrator *calibrator)
{
	this->calibrator = calibrator;
}

IMUTracker::~IMUTracker(void)
{
}

void IMUTracker::TrackCamera(CameraTrackingState *trackingState, const View *view)
{
	calibrator->RegisterMeasurement(((ViewIMU*)view)->imu->R);

	trackingState->pose_d->SetR(calibrator->GetDifferentialRotationChange() * trackingState->pose_d->GetR());
}
