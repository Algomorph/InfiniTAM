// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "DepthTracker.h"
#include "../../../ORUtils/Cholesky.h"

#include <math.h>

using namespace ITMLib;

DepthTracker::DepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                           float terminationThreshold, float failureDetectorThreshold, const ImageProcessingEngineInterface *lowLevelEngine, MemoryDeviceType memoryType)
{
	viewHierarchy = new ImageHierarchy<TemplatedHierarchyLevel<FloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	sceneHierarchy = new ImageHierarchy<VolumeHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);

	this->noIterationsPerLevel = new int[noHierarchyLevels];
	this->distThresh = new float[noHierarchyLevels];

	SetupLevels(noHierarchyLevels * 2, 2, 0.01f, 0.002f);

	this->lowLevelEngine = lowLevelEngine;

	this->terminationThreshold = terminationThreshold;

	map = new ORUtils::HomkerMap(2);
	svmClassifier = new ORUtils::SVMClassifier(map->getDescriptorSize(4));

	//all below obtained from dataset in matlab
	float w[20];
	w[0] = -3.15813f; w[1] = -2.38038f; w[2] = 1.93359f; w[3] = 1.56642f; w[4] = 1.76306f;
	w[5] = -0.747641f; w[6] = 4.41852f; w[7] = 1.72048f; w[8] = -0.482545f; w[9] = -5.07793f;
	w[10] = 1.98676f; w[11] = -0.45688f; w[12] = 2.53969f; w[13] = -3.50527f; w[14] = -1.68725f;
	w[15] = 2.31608f; w[16] = 5.14778f; w[17] = 2.31334f; w[18] = -14.128f; w[19] = 6.76423f;

	float b = 9.334260e-01f + failureDetectorThreshold;

	mu = Vector4f(-34.9470512137603f, -33.1379108518478f, 0.195948598235857f, 0.611027292662361f);
	sigma = Vector4f(68.1654461020426f, 60.6607826748643f, 0.00343068557187040f, 0.0402595570918749f);

	svmClassifier->SetVectors(w, b);
}

DepthTracker::~DepthTracker()
{
	delete this->viewHierarchy;
	delete this->sceneHierarchy;

	delete[] this->noIterationsPerLevel;
	delete[] this->distThresh;

	delete map;
	delete svmClassifier;
}

void DepthTracker::SetupLevels(int numIterCoarse, int numIterFine, float distThreshCoarse, float distThreshFine)
{
	int noHierarchyLevels = viewHierarchy->GetNoLevels();

	if ((numIterCoarse != -1) && (numIterFine != -1)) {
		float step = (float)(numIterCoarse - numIterFine) / (float)(noHierarchyLevels - 1);
		float val = (float)numIterCoarse;
		for (int levelId = noHierarchyLevels - 1; levelId >= 0; levelId--) {
			this->noIterationsPerLevel[levelId] = (int)round(val);
			val -= step;
		}
	}
	if ((distThreshCoarse >= 0.0f) && (distThreshFine >= 0.0f)) {
		float step = (float)(distThreshCoarse - distThreshFine) / (float)(noHierarchyLevels - 1);
		float val = distThreshCoarse;
		for (int levelId = noHierarchyLevels - 1; levelId >= 0; levelId--) {
			this->distThresh[levelId] = val;
			val -= step;
		}
	}
}

void DepthTracker::SetEvaluationData(CameraTrackingState *trackingState, const View* view)
{
	this->trackingState = trackingState;
	this->view = view;

	sceneHierarchy->GetLevel(0)->intrinsics = view->calibration_information.intrinsics_d.projectionParamsSimple.all;
	viewHierarchy->GetLevel(0)->intrinsics = view->calibration_information.intrinsics_d.projectionParamsSimple.all;

	// the image hierarchy allows pointers to external data at level 0
	// TODO: somehow fix this cast that removes const modifier... TRAVESTY!
	viewHierarchy->GetLevel(0)->data = (FloatImage*)&view->depth;
	sceneHierarchy->GetLevel(0)->pointsMap = trackingState->point_cloud->locations;
	sceneHierarchy->GetLevel(0)->normalsMap = trackingState->point_cloud->colors;

	scenePose = trackingState->pose_pointCloud->GetM();
}

void DepthTracker::PrepareForEvaluation()
{
	for (int i = 1; i < viewHierarchy->GetNoLevels(); i++)
	{
		TemplatedHierarchyLevel<FloatImage> *currentLevelView = viewHierarchy->GetLevel(i);
		TemplatedHierarchyLevel<FloatImage> *previousLevelView = viewHierarchy->GetLevel(i - 1);
		lowLevelEngine->FilterSubsampleWithHoles(*currentLevelView->data, *previousLevelView->data);
		currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;

		VolumeHierarchyLevel *currentLevelScene = sceneHierarchy->GetLevel(i);
		VolumeHierarchyLevel *previousLevelScene = sceneHierarchy->GetLevel(i - 1);
		//image_processing_engine->FilterSubsampleWithHoles(currentLevelScene->pointsMap, previousLevelScene->pointsMap);
		//image_processing_engine->FilterSubsampleWithHoles(currentLevelScene->normalsMap, previousLevelScene->normalsMap);
		currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

void DepthTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
	this->iterationType = viewHierarchy->GetLevel(levelId)->iterationType;
	this->sceneHierarchyLevel = sceneHierarchy->GetLevel(0);
	this->viewHierarchyLevel = viewHierarchy->GetLevel(levelId);
}

void DepthTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (shortIteration)
	{
		float smallHessian[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);
	}
	else
	{
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);
	}
}

bool DepthTracker::HasConverged(float *step) const
{
	float stepLength = 0.0f;
	for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

	if (sqrt(stepLength) / 6 < terminationThreshold) return true; //converged

	return false;
}

void DepthTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = 0.0f; step[4] = 0.0f; step[5] = 0.0f;
		break;
	case TRACKER_ITERATION_TRANSLATION:
		step[0] = 0.0f; step[1] = 0.0f; step[2] = 0.0f;
		step[3] = (float)(delta[0]); step[4] = (float)(delta[1]); step[5] = (float)(delta[2]);
		break;
	default:
	case TRACKER_ITERATION_BOTH:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = (float)(delta[3]); step[4] = (float)(delta[4]); step[5] = (float)(delta[5]);
		break;
	}

	Matrix4f Tinc;

	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	para_new = Tinc * para_old;
}

void DepthTracker::UpdatePoseQuality(int old_valid_points_count, float *hessian_good, float f_old)
{
	size_t point_count = viewHierarchy->GetLevel(0)->data->size();

	int max_valid_point_count = lowLevelEngine->CountValidDepths(view->depth);

	float normFactor_v1 = (float)old_valid_points_count / (float)point_count;
	float normFactor_v2 = (float)old_valid_points_count / (float)max_valid_point_count;

	float det = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		ORUtils::Cholesky cholA(hessian_good, 6);
		det = cholA.Determinant();
		if (isnan(det)) det = 0.0f;
	}

	float det_norm_v1 = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v1;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v1 = cholA.Determinant();
		if (isnan(det_norm_v1)) det_norm_v1 = 0.0f;
	}

	float det_norm_v2 = 0.0f;
	if (iterationType == TRACKER_ITERATION_BOTH) {
		float h[6 * 6];
		for (int i = 0; i < 6 * 6; ++i) h[i] = hessian_good[i] * normFactor_v2;
		ORUtils::Cholesky cholA(h, 6);
		det_norm_v2 = cholA.Determinant();
		if (isnan(det_norm_v2)) det_norm_v2 = 0.0f;
	}

	float finalResidual_v2 = sqrt(((float)old_valid_points_count * f_old + (float)(max_valid_point_count - old_valid_points_count) * distThresh[0]) / (float)max_valid_point_count);
	float percentageInliers_v2 = (float)old_valid_points_count / (float)max_valid_point_count;

	trackingState->trackerResult = CameraTrackingState::TRACKING_FAILED;

	if (max_valid_point_count != 0 && point_count != 0 && det_norm_v1 > 0 && det_norm_v2 > 0) {
		Vector4f inputVector(log(det_norm_v1), log(det_norm_v2), finalResidual_v2, percentageInliers_v2);

		Vector4f normalisedVector = (inputVector - mu) / sigma;

		float mapped[20];
		map->evaluate(mapped, normalisedVector.values, 4);

		float score = svmClassifier->Classify(mapped);

		if (score > 0) trackingState->trackerResult = CameraTrackingState::TRACKING_GOOD;
		else if (score > -10.0f) trackingState->trackerResult = CameraTrackingState::TRACKING_POOR;
	}
}

void DepthTracker::TrackCamera(CameraTrackingState *trackingState, const View *view)
{
	this->SetEvaluationData(trackingState, view);
	this->PrepareForEvaluation();

	float f_old = 1e10, f_new;
	int noValidPoints_new;
	int noValidPoints_old = 0;

	float hessian_good[6 * 6], hessian_new[6 * 6], A[6 * 6];
	float nabla_good[6], nabla_new[6];
	float step[6];

	for (int i = 0; i < 6 * 6; ++i) hessian_good[i] = 0.0f;
	for (int i = 0; i < 6; ++i) nabla_good[i] = 0.0f;

	for (int levelId = viewHierarchy->GetNoLevels() - 1; levelId >= 0; levelId--)
	{
		this->SetEvaluationParams(levelId);
		if (iterationType == TRACKER_ITERATION_NONE) continue;

		Matrix4f approxInvPose = trackingState->pose_d->GetInvM();
		ORUtils::SE3Pose lastKnownGoodPose(*(trackingState->pose_d));
		f_old = 1e20f;
		noValidPoints_old = 0;
		float lambda = 1.0;

		for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
		{
			// evaluate error function and gradients
			noValidPoints_new = this->ComputeGandH(f_new, nabla_new, hessian_new, approxInvPose);

			// check if error increased. If so, revert
			if ((noValidPoints_new <= 0) || (f_new > f_old)) {
				trackingState->pose_d->SetFrom(&lastKnownGoodPose);
				approxInvPose = trackingState->pose_d->GetInvM();
				lambda *= 10.0f;
			}
			else {
				lastKnownGoodPose.SetFrom(trackingState->pose_d);
				f_old = f_new;
				noValidPoints_old = noValidPoints_new;

				for (int i = 0; i < 6 * 6; ++i) hessian_good[i] = hessian_new[i] / noValidPoints_new;
				for (int i = 0; i < 6; ++i) nabla_good[i] = nabla_new[i] / noValidPoints_new;
				lambda /= 10.0f;
			}
			for (int i = 0; i < 6 * 6; ++i) A[i] = hessian_good[i];
			for (int i = 0; i < 6; ++i) A[i + i * 6] *= 1.0f + lambda;

			// compute_allocated a new step and make sure we've got an SE3
			ComputeDelta(step, nabla_good, A, iterationType != TRACKER_ITERATION_BOTH);
			ApplyDelta(approxInvPose, step, approxInvPose);
			trackingState->pose_d->SetInvM(approxInvPose);
			trackingState->pose_d->Coerce();
			approxInvPose = trackingState->pose_d->GetInvM();

			// if step is small, assume it's going to decrease the error and finish
			if (HasConverged(step)) break;
		}
	}

	this->UpdatePoseQuality(noValidPoints_old, hessian_good, f_old);
}
