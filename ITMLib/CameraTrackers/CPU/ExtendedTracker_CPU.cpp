// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ExtendedTracker_CPU.h"
#include "../Shared/ExtendedTracker_Shared.h"

using namespace ITMLib;

ExtendedTracker_CPU::ExtendedTracker_CPU(Vector2i imgSize_d,
                                         Vector2i imgSize_rgb,
                                         bool useDepth,
                                         bool useColour,
                                         float colourWeight,
                                         TrackerIterationType *trackingRegime,
                                         int noHierarchyLevels,
                                         float terminationThreshold,
                                         float failureDetectorThreshold,
                                         float viewFrustum_min,
                                         float viewFrustum_max,
                                         float minColourGradient,
                                         float tukeyCutOff,
                                         int framesToSkip,
                                         int framesToWeight,
                                         const ImageProcessingEngineInterface *lowLevelEngine)
	: ExtendedTracker(imgSize_d,
	                  imgSize_rgb,
	                  useDepth,
	                  useColour,
	                  colourWeight,
	                  trackingRegime,
	                  noHierarchyLevels,
	                  terminationThreshold,
	                  failureDetectorThreshold,
	                  viewFrustum_min,
	                  viewFrustum_max,
	                  minColourGradient,
	                  tukeyCutOff,
	                  framesToSkip,
	                  framesToWeight,
	                  lowLevelEngine,
	                  MEMORYDEVICE_CPU)
{ }

ExtendedTracker_CPU::~ExtendedTracker_CPU() { }

/**
 * \brief Compute gradient and Hessian matrix of the new depth fitted to global data
 * \param f [out] the local estimates to the optimization objective function //TODO: verify --Greg(?)
 * \param nabla [out] local gradient based on depth
 * \param hessian [out] local hessian approximation based on depth
 * \param approxInvPose current inverse camera pose estimation
 * \return number of points for which the current computation is valid
 */
int ExtendedTracker_CPU::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = point_cloud_hierarchy_level_depth->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = point_cloud_hierarchy_level_depth->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = point_cloud_hierarchy_level_depth->intrinsics;
	Vector2i sceneImageSize = point_cloud_hierarchy_level_depth->pointsMap->dimensions;

	const float *depth = view_hierarchy_level_depth->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = view_hierarchy_level_depth->intrinsics;
	Vector2i viewImageSize = view_hierarchy_level_depth->depth->dimensions;

	if (current_iteration_type == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (current_iteration_type == TRACKER_ITERATION_ROTATION)
						   || (current_iteration_type == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint;

		float depthWeight;

		if (frames_processed < 100)
		{
			switch (current_iteration_type)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth<true, true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                            viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                            near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth<true, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                             viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                             near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth<false, false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                              viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                              near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}
		else
		{
			switch (current_iteration_type)
			{
			case TRACKER_ITERATION_ROTATION:
				isValidPoint = computePerPointGH_exDepth<true, true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                           viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                           near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			case TRACKER_ITERATION_TRANSLATION:
				isValidPoint = computePerPointGH_exDepth<true, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                            viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                            near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			case TRACKER_ITERATION_BOTH:
				isValidPoint = computePerPointGH_exDepth<false, false, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight,
				                                                             viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scene_pose, pointsMap, normalsMap, level_distance_thresholds[current_level_id],
				                                                             near_clipping_distance, far_clipping_distance, tukey_cutoff, frames_to_skip, frames_to_weight);
				break;
			default:
				isValidPoint = false;
				break;
			}
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	// Copy the lower triangular part of the matrix.
	for (int r = 0, counter = 0; r < noPara; r++)
		for (int c = 0; c <= r; c++, counter++)
			hessian[r + c * 6] = sumHessian[counter];

	// Transpose to fill the upper triangle.
	for (int r = 0; r < noPara; ++r)
		for (int c = r + 1; c < noPara; c++)
			hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	f = sumF;

	return noValidPoints;
}

int ExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	const Vector2i viewImageSize_depth = view_hierarchy_level_depth->depth->dimensions;
	const Vector2i viewImageSize_rgb = view_hierarchy_level_intensity->intensity_prev->dimensions;

	const Vector4f *points_curr = reprojected_points_level->data->GetData(MEMORYDEVICE_CPU);
	const float *intensities_prev = view_hierarchy_level_intensity->intensity_prev->GetData(MEMORYDEVICE_CPU);
	const float *intensities_current = projected_intensity_level->data->GetData(MEMORYDEVICE_CPU);
	const Vector2f *gradients = view_hierarchy_level_intensity->gradients->GetData(MEMORYDEVICE_CPU);

	Vector4f projParams_rgb = view_hierarchy_level_intensity->intrinsics;
	Vector4f projParams_depth = view_hierarchy_level_depth->intrinsics;

	if (current_iteration_type == TRACKER_ITERATION_NONE) return 0;

	bool shortIteration = (current_iteration_type == TRACKER_ITERATION_ROTATION)
						   || (current_iteration_type == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF;
	int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);

	for (int y = 0; y < viewImageSize_depth.y; y++) for (int x = 0; x < viewImageSize_depth.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		bool isValidPoint = false;

		switch (current_iteration_type)
		{
		case TRACKER_ITERATION_ROTATION:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<true, true>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depth_to_color_camera_transform * scene_pose,
					level_color_thresholds[current_level_id],
					min_color_gradient,
					near_clipping_distance,
					far_clipping_distance,
					tukey_cutoff
					);
			break;
		case TRACKER_ITERATION_TRANSLATION:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<true, false>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depth_to_color_camera_transform * scene_pose,
					level_color_thresholds[current_level_id],
					min_color_gradient,
					near_clipping_distance,
					far_clipping_distance,
					tukey_cutoff
					);
			break;
		case TRACKER_ITERATION_BOTH:
			isValidPoint = computePerPointGH_exRGB_inv_Ab<false, false>(
					localF,
					localNabla,
					localHessian,
					x,
					y,
					points_curr,
					intensities_current,
					intensities_prev,
					gradients,
					viewImageSize_depth,
					viewImageSize_rgb,
					projParams_depth,
					projParams_rgb,
					approxInvPose,
					depth_to_color_camera_transform * scene_pose,
					level_color_thresholds[current_level_id],
					min_color_gradient,
					near_clipping_distance,
					far_clipping_distance,
					tukey_cutoff
					);
			break;
		default:
			isValidPoint = false;
			break;
		}

		if (isValidPoint)
		{
			noValidPoints++;
			sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	// Copy the lower triangular part of the matrix.
	for (int r = 0, counter = 0; r < noPara; r++)
		for (int c = 0; c <= r; c++, counter++)
			hessian[r + c * 6] = sumHessian[counter];

	// Transpose to fill the upper triangle.
	for (int r = 0; r < noPara; ++r)
		for (int c = r + 1; c < noPara; c++)
			hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, sumNabla, noPara * sizeof(float));

	f = sumF;

	return noValidPoints;
}

void ExtendedTracker_CPU::ProjectCurrentIntensityFrame(Float4Image *points_out,
                                                       FloatImage *intensity_out,
                                                       const FloatImage *intensity_in,
                                                       const FloatImage *depth_in,
                                                       const Vector4f &intrinsics_depth,
                                                       const Vector4f &intrinsics_rgb,
                                                       const Matrix4f &scenePose)
{
	const Vector2i imageSize_rgb = intensity_in->dimensions;
	const Vector2i imageSize_depth = depth_in->dimensions; // Also the size of the projected image

	points_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per Run.
	intensity_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per Run.

	const float *depths = depth_in->GetData(MEMORYDEVICE_CPU);
	const float *intensityIn = intensity_in->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsOut = points_out->GetData(MEMORYDEVICE_CPU);
	float *intensityOut = intensity_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imageSize_depth.y; y++) for (int x = 0; x < imageSize_depth.x; x++)
		projectPoint_exRGB(x, y, pointsOut, intensityOut, intensityIn, depths, imageSize_rgb, imageSize_depth, intrinsics_rgb, intrinsics_depth, scenePose);
}
