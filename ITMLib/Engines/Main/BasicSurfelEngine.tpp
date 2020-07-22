// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "BasicSurfelEngine.h"

#include "../ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ViewBuilding/ViewBuilderFactory.h"
#include "../Rendering/SurfelVisualizationEngineFactory.h"
#include "../../CameraTrackers/CameraTrackerFactory.h"

#include "../../../ORUtils/NVTimer.h"
#include "../../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

using namespace ITMLib;

template<typename TSurfel>
BasicSurfelEngine<TSurfel>::BasicSurfelEngine(const RGBD_CalibrationInformation& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
		: trackingActive(this->parameters.enable_rigid_alignment){
	auto& settings = configuration::get();

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	MemoryDeviceType memoryType = settings.device_type;
	this->surfelScene = new SurfelScene<TSurfel>(&settings.general_surfel_volume_parameters, memoryType);

	const MemoryDeviceType deviceType = settings.device_type;

	lowLevelEngine = ImageProcessingEngineFactory::Build(deviceType);
	viewBuilder = ViewBuilderFactory::Build(calib, deviceType);
	surfelVisualizationEngine = SurfelVisualizationEngineFactory::Build<TSurfel>(deviceType);

	denseSurfelMapper = new DenseSurfelMapper<TSurfel>(imgSize_d, deviceType);
	this->surfelScene->Reset();

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = CameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, lowLevelEngine, imuCalibrator,
	                                                settings.general_voxel_volume_parameters);
	trackingController = new CameraTrackingController(tracker);

	Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	surfelRenderState_live = new SurfelRenderState(trackedImageSize,
	                                               settings.general_surfel_volume_parameters.supersampling_factor);
	surfelRenderState_freeview = nullptr; //will be created if needed

	trackingState = new CameraTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = nullptr; // will be allocated by the view builder

	if (settings.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings.general_voxel_volume_parameters.near_clipping_distance,
		                                                                       settings.general_voxel_volume_parameters.far_clipping_distance),
		                                                   0.2f, 500, 4);
	else relocaliser = nullptr;

	kfRaycast = new UChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
}

template<typename TSurfel>
BasicSurfelEngine<TSurfel>::~BasicSurfelEngine() {
	delete surfelRenderState_live;
	delete surfelRenderState_freeview;

	delete surfelScene;

	delete denseSurfelMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != nullptr) delete view;

	delete surfelVisualizationEngine;

	if (relocaliser != nullptr) delete relocaliser;
	delete kfRaycast;
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::SaveVolumeToMesh(const std::string& path) {
	// Not yet implemented for surfel volumes
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::SaveToFile() {
	// Not yet implemented for surfel volumes
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::LoadFromFile() {
	// Not yet implemented for surfel volumes
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::ResetAll() {
	surfelScene->Reset();
	trackingState->Reset();
}

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
	int variant = 0;
	if
		((matrix[4]>-matrix[8]) && (matrix[0]>-matrix[4]) && (matrix[0]>-matrix[8]))
	{
		variant = 0;
	}
	else if ((matrix[4]<-matrix[8]) && (matrix[0]>
		matrix[4]) && (matrix[0]> matrix[8])) {
		variant = 1;
	}
	else if ((matrix[4]> matrix[8]) && (matrix[0]<
		matrix[4]) && (matrix[0]<-matrix[8])) {
		variant = 2;
	}
	else if ((matrix[4]<
		matrix[8]) && (matrix[0]<-matrix[4]) && (matrix[0]< matrix[8])) {
		variant = 3;
	}
	return variant;
}

static void QuaternionFromRotationMatrix(const double *matrix, double *q) {
	/* taken from "James Diebel. Representing Attitude: Euler
	Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
	University, Palo Alto, CA."
	*/

	// choose the numerically best variant...
	int variant = QuaternionFromRotationMatrix_variant(matrix);
	double denom = 1.0;
	if (variant == 0) {
		denom += matrix[0] + matrix[4] + matrix[8];
	}
	else {
		int tmp = variant * 4;
		denom += matrix[tmp - 4];
		denom -= matrix[tmp % 12];
		denom -= matrix[(tmp + 4) % 12];
	}
	denom = sqrt(denom);
	q[variant] = 0.5*denom;

	denom *= 2.0;
	switch (variant) {
	case 0:
		q[1] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[6] - matrix[2]) / denom;
		q[3] = (matrix[1] - matrix[3]) / denom;
		break;
	case 1:
		q[0] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[6] + matrix[2]) / denom;
		break;
	case 2:
		q[0] = (matrix[6] - matrix[2]) / denom;
		q[1] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[5] + matrix[7]) / denom;
		break;
	case 3:
		q[0] = (matrix[1] - matrix[3]) / denom;
		q[1] = (matrix[6] + matrix[2]) / denom;
		q[2] = (matrix[5] + matrix[7]) / denom;
		break;
	}

	if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}
#endif

template<typename TSurfel>
CameraTrackingState::TrackingResult
BasicSurfelEngine<TSurfel>::ProcessFrame(UChar4Image* rgbImage, ShortImage* rawDepthImage,
                                         IMUMeasurement* imuMeasurement) {
	auto& settings = configuration::get();
	// prepare image and turn it into a depth image
	if (imuMeasurement == nullptr)
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, false, true);
	else
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, imuMeasurement, false, true);

	if (!mainProcessingActive) return CameraTrackingState::TRACKING_FAILED;

	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
	if (trackingActive) trackingController->Track(trackingState, view);

	CameraTrackingState::TrackingResult trackerResult = CameraTrackingState::TRACKING_GOOD;
	switch (settings.behavior_on_failure) {
		case configuration::FAILUREMODE_RELOCALIZE:
			trackerResult = trackingState->trackerResult;
			break;
		case configuration::FAILUREMODE_STOP_INTEGRATION:
			if (trackingState->trackerResult != CameraTrackingState::TRACKING_FAILED)
				trackerResult = trackingState->trackerResult;
			else trackerResult = CameraTrackingState::TRACKING_POOR;
			break;
		default:
			break;
	}

	//relocalisation

#if 0 //TODO: explain not compiled block in comment (see below)? --Greg(GitHub:Algomorph)
	int addKeyframeIdx = -1;
#endif
	if (settings.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE) {
		if (trackerResult == CameraTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

		int NN;
		float distances;
		view->depth.UpdateHostFromDevice();

		//find and add keyframe, if necessary
		bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances,
		                                                  trackerResult == CameraTrackingState::TRACKING_GOOD &&
		                                                  relocalisationCount == 0);

		//frame not added and tracking failed -> we need to relocalise
		if (!hasAddedKeyframe && trackerResult == CameraTrackingState::TRACKING_FAILED) {
			relocalisationCount = 10;

			// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
			view->rgb_prev->Clear();

			const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocaliser->RetrievePose(NN);
			trackingState->pose_d->SetFrom(&keyframe.pose);

			trackingController->Prepare(trackingState, surfelScene, view, surfelVisualizationEngine,
			                            surfelRenderState_live);
			surfelVisualizationEngine->FindSurfaceSuper(surfelScene, trackingState->pose_d, &view->calibration_information.intrinsics_d,
			                                            USR_RENDER, surfelRenderState_live);
			trackingController->Track(trackingState, view);

			trackerResult = trackingState->trackerResult;
		}
	}

	//bool didFusion = false; //TODO: set but not used. Remove? --Greg(github:Algomorph)
	if ((trackerResult == CameraTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		// fusion
		denseSurfelMapper->ProcessFrame(view, trackingState, surfelScene, surfelRenderState_live);
		//didFusion = true; //TODO: set but not used. Remove? --Greg(github:Algomorph)
		if (framesProcessed > 50) trackingInitialised = true;

		framesProcessed++;
	}

	if (trackerResult == CameraTrackingState::TRACKING_GOOD || trackerResult == CameraTrackingState::TRACKING_POOR) {
		// raycast to renderState_canonical for tracking and free Rendering
		trackingController->Prepare(trackingState, surfelScene, view, surfelVisualizationEngine,
		                            surfelRenderState_live);
		surfelVisualizationEngine->FindSurfaceSuper(surfelScene, trackingState->pose_d, &view->calibration_information.intrinsics_d,
		                                            USR_RENDER, surfelRenderState_live);

#if 0 //TODO: explain not compiled block in comment? --Greg(GitHub:Algomorph)
		if (addKeyframeIdx >= 0)
		{
			ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
				settings.device_type == MEMORYDEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

			keyframe_raycast->SetFrom(renderState_canonical->raycastImage, memoryCopyDirection);
		}
#endif
	} else *trackingState->pose_d = oldPose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
	const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().m[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif

	return trackerResult;
}

template<typename TSurfel>
Vector2i BasicSurfelEngine<TSurfel>::GetImageSize() const {
	return surfelRenderState_live->GetIndexImage()->dimensions;
}

template<typename TSurfel>
typename SurfelVisualizationEngine<TSurfel>::RenderImageType
BasicSurfelEngine<TSurfel>::ToSurfelImageType(GetImageType getImageType) {
	switch (getImageType) {
		case InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
			return SurfelVisualizationEngine<TSurfel>::RENDER_COLOUR;
		case InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
			return SurfelVisualizationEngine<TSurfel>::RENDER_NORMAL;
		case InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
		case InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
			return SurfelVisualizationEngine<TSurfel>::RENDER_CONFIDENCE;
		default:
			return SurfelVisualizationEngine<TSurfel>::RENDER_LAMBERTIAN;
	}
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::GetImage(UChar4Image* out, GetImageType getImageType, ORUtils::SE3Pose* pose,
                                          Intrinsics* intrinsics) {

	auto& settings = configuration::get();
	if (view == nullptr) return;

	out->Clear();

	switch (getImageType) {
		case BasicSurfelEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb.dimensions);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
			break;
		case BasicSurfelEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth.dimensions);
			if (settings.device_type == MEMORYDEVICE_CUDA) view->depth.UpdateHostFromDevice();
			IRenderingEngine::DepthToUchar4(out, view->depth);
			break;
		case BasicSurfelEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		case BasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case BasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case BasicSurfelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
			const bool useRadii = true;
			surfelVisualizationEngine->FindSurface(surfelScene, trackingState->pose_d, &view->calibration_information.intrinsics_d,
			                                       useRadii, USR_DONOTRENDER, surfelRenderState_live);
			surfelVisualizationEngine->RenderImage(surfelScene, trackingState->pose_d, surfelRenderState_live, out,
			                                       ToSurfelImageType(getImageType));
			out->UpdateHostFromDevice();
			break;
		}
		case BasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case BasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case BasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case BasicSurfelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			if (!surfelRenderState_freeview)
				surfelRenderState_freeview = new SurfelRenderState(view->depth.dimensions,
				                                                   surfelScene->GetParams().supersampling_factor);
			const bool useRadii = true;
			surfelVisualizationEngine->FindSurface(surfelScene, pose, intrinsics, useRadii, USR_DONOTRENDER,
			                                       surfelRenderState_freeview);
			surfelVisualizationEngine->RenderImage(surfelScene, pose, surfelRenderState_freeview, out,
			                                       ToSurfelImageType(getImageType));
			out->UpdateHostFromDevice();
			break;
		}
		case FusionAlgorithm::InfiniTAM_IMAGE_UNKNOWN: break;
		case InfiniTAM_IMAGE_FREECAMERA_CANONICAL:break;
	}
}

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOnTracking() { trackingActive = true; }

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOffTracking() { trackingActive = false; }

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOnIntegration() { fusionActive = true; }

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOffIntegration() { fusionActive = false; }

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOnMainProcessing() { mainProcessingActive = true; }

template<typename TSurfel>
void BasicSurfelEngine<TSurfel>::TurnOffMainProcessing() { mainProcessingActive = false; }
