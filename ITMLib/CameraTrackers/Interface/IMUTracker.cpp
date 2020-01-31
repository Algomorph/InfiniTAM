// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "IMUTracker.h"

#include "../../Objects/Views/ITMViewIMU.h"
using namespace ITMLib;

IMUTracker::IMUTracker(ITMIMUCalibrator *calibrator)
{
	this->calibrator = calibrator;
}

IMUTracker::~IMUTracker(void)
{
}

void IMUTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	calibrator->RegisterMeasurement(((ITMViewIMU*)view)->imu->R);

	trackingState->pose_d->SetR(calibrator->GetDifferentialRotationChange() * trackingState->pose_d->GetR());
}
