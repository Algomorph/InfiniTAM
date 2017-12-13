// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMKillingEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS
//#define SAVE_DEBUG_IMAGES
//#define LOAD_DEBUG_IMAGES
//#define FIX_DEBUG_TRACKER

#include "../../ORUtils/FileUtils.h"

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::ITMKillingEngine(const ITMLibSettings* settings,
                                                                        const ITMRGBDCalib& calib, Vector2i imgSize_rgb,
                                                                        Vector2i imgSize_d) {
	this->settings = settings;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	MemoryDeviceType memoryType = settings->GetMemoryType();
	this->canonical_scene = new ITMScene<TVoxelCanonical, TIndex>(&settings->sceneParams, settings->swappingMode ==
	                                                                                      ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                              memoryType);
	this->live_scene = new ITMScene<ITMVoxelLive, TIndex>(&settings->sceneParams, settings->swappingMode ==
	                                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                      memoryType);
	const ITMLibSettings::DeviceType deviceType = settings->deviceType;

	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxelCanonical, TIndex>(deviceType);

	meshingEngine = NULL;
	if (settings->createMeshingEngine)
		meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxelCanonical, TIndex>(deviceType);

	denseMapper = new ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>(settings);
	denseMapper->ResetScene(canonical_scene);
	denseMapper->ResetLiveScene(live_scene);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator,
	                                             canonical_scene->sceneParams);
	trackingController = new ITMTrackingController(tracker, settings);

	Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	renderState_live = ITMRenderStateFactory<TIndex>::CreateRenderState(trackedImageSize, canonical_scene->sceneParams,
	                                                                    memoryType);
	renderState_freeview = NULL; //will be created if needed

	trackingState = new ITMTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder

	if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min,
		                                                                       settings->sceneParams.viewFrustum_max),
		                                                   0.2f, 500, 4);
	else relocaliser = NULL;

	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
	fusionInitialized = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::~ITMKillingEngine() {
	delete renderState_live;
	if (renderState_freeview != NULL) delete renderState_freeview;

	delete canonical_scene;
	delete live_scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (relocaliser != NULL) delete relocaliser;
	delete kfRaycast;

	if (meshingEngine != NULL) delete meshingEngine;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::SaveSceneToMesh(const char* objFileName) {
	if (meshingEngine == NULL) return;

	ITMMesh* mesh = new ITMMesh(settings->GetMemoryType());

	meshingEngine->MeshScene(mesh, canonical_scene);
	mesh->WriteSTL(objFileName);

	delete mesh;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::SaveToFile() {
	// throws error if any of the saves fail

	std::string saveOutputDirectory = "State/";
	std::string relocaliserOutputDirectory = saveOutputDirectory + "Relocaliser/", sceneOutputDirectory =
			saveOutputDirectory + "Scene/";

	MakeDir(saveOutputDirectory.c_str());
	MakeDir(relocaliserOutputDirectory.c_str());
	MakeDir(sceneOutputDirectory.c_str());

	if (relocaliser) relocaliser->SaveToDirectory(relocaliserOutputDirectory);

	canonical_scene->SaveToDirectory(sceneOutputDirectory);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::LoadFromFile() {
	std::string saveInputDirectory = "State/";
	std::string relocaliserInputDirectory = saveInputDirectory + "Relocaliser/", sceneInputDirectory =
			saveInputDirectory + "Scene/";

	////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
	////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

	this->resetAll();

	try // load relocaliser
	{
		FernRelocLib::Relocaliser<float>* relocaliser_temp = new FernRelocLib::Relocaliser<float>(view->depth->noDims,
		                                                                                          Vector2f(
				                                                                                          settings->sceneParams.viewFrustum_min,
				                                                                                          settings->sceneParams.viewFrustum_max),
		                                                                                          0.2f, 500, 4);

		relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

		delete relocaliser;
		relocaliser = relocaliser_temp;
	}
	catch (std::runtime_error& e) {
		throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
	}

	try // load canonical_scene
	{
		canonical_scene->LoadFromDirectory(sceneInputDirectory);
	}
	catch (std::runtime_error& e) {
		denseMapper->ResetScene(canonical_scene);
		throw std::runtime_error("Could not load canonical_scene:" + std::string(e.what()));
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::resetAll() {
	denseMapper->ResetScene(canonical_scene);
	denseMapper->ResetLiveScene(live_scene);
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

#if defined(SAVE_DEBUG_IMAGES) || defined(LOAD_DEBUG_IMAGES)
static int imageCounter = 0;
static const std::string imageOutFolder = "/media/algomorph/Data/4dmseg/Killing/debug_images";
#endif


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMTrackingState::TrackingResult
ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(ITMUChar4Image* rgbImage,
                                                                    ITMShortImage* rawDepthImage,
                                                                    ITMIMUMeasurement* imuMeasurement) {
	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);
#ifdef SAVE_DEBUG_IMAGES
	std::string fullPath = imageOutFolder + "/depth_" + std::to_string(imageCounter) +".png";
	SaveImageToFile(rawDepthImage, fullPath.c_str());
	imageCounter++;
#endif

#ifdef LOAD_DEBUG_IMAGES
	std::string fullPath = imageOutFolder + "/depth_" + std::to_string(imageCounter) +".png";
	ITMShortImage * prevDepthImage = new ITMShortImage(true, false);
	ReadImageFromFile(prevDepthImage, fullPath.c_str());
	if(*prevDepthImage == *rawDepthImage){
		std::cout << "equal" << std::endl;
	}else{
		std::cout << "not equal" << std::endl;
	}
	delete prevDepthImage;
	imageCounter++;
#endif

	if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;

	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
	if (trackingActive) trackingController->Track(trackingState, view);
#ifdef FIX_DEBUG_TRACKER
	float track_result_contents[] =
			{1, -0.00030433, 0.000681089, -0.000994006,
			 0.000305798, 0.999998, -0.0021771, 0.00294407,
			 -0.000680431, 0.0021773, 0.999997, -8.74213e-05,
			 0, 0, 0, 1};
	Matrix4f fixedTrackedModelview;
	fixedTrackedModelview.setValues(track_result_contents);
	trackingState->pose_d->SetM(fixedTrackedModelview);
#endif


	ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;
	switch (settings->behaviourOnFailure) {
		case ITMLibSettings::FAILUREMODE_RELOCALISE:
			trackerResult = trackingState->trackerResult;
			break;
		case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
			if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
				trackerResult = trackingState->trackerResult;
			else trackerResult = ITMTrackingState::TRACKING_POOR;
			break;
		default:
			break;
	}

	//relocalisation
	int addKeyframeIdx = -1;
	if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE) {
		if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

		int NN;
		float distances;
		view->depth->UpdateHostFromDevice();

		//find and add keyframe, if necessary
		bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances,
		                                                  trackerResult == ITMTrackingState::TRACKING_GOOD &&
		                                                  relocalisationCount == 0);

		//frame not added and tracking failed -> we need to relocalise
		if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED) {
			relocalisationCount = 10;

			// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
			view->rgb_prev->Clear();

			const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocaliser->RetrievePose(NN);
			trackingState->pose_d->SetFrom(&keyframe.pose);

			//denseMapper->UpdateVisibleList(view, trackingState, live_scene, renderState_live, true);
			denseMapper->UpdateVisibleList(view, trackingState, canonical_scene, renderState_live, true);
			trackingController->Prepare(trackingState, canonical_scene, view, visualisationEngine, renderState_live);
			trackingController->Track(trackingState, view);

			trackerResult = trackingState->trackerResult;
		}
	}

	bool didFusion = false;
	if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		if (!fusionInitialized) {
			// fusion
			denseMapper->ProcessInitialFrame(view, trackingState, canonical_scene, renderState_live);
			fusionInitialized = true;
		} else {
			// fusion
			denseMapper->ProcessFrame(view, trackingState, canonical_scene, live_scene, renderState_live);
		}
		didFusion = true;
		if (framesProcessed > 50) trackingInitialised = true;

		framesProcessed++;
	}

	if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR) {
		if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, canonical_scene, renderState_live);

		// raycast to renderState_live for tracking and free visualisation
		trackingController->Prepare(trackingState, canonical_scene, view, visualisationEngine, renderState_live);

		if (addKeyframeIdx >= 0) {
			ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
					settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA
					                                                    : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

			kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
		}
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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
Vector2i ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::GetImageSize(void) const {
	return renderState_live->raycastImage->noDims;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::GetImage(ITMUChar4Image* out, GetImageType getImageType,
                                                                     ORUtils::SE3Pose* pose,
                                                                     ITMIntrinsics* intrinsics) {
	if (view == NULL) return;

	out->Clear();

	switch (getImageType) {
		case ITMKillingEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
			break;
		case ITMKillingEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<TVoxelCanonical, TIndex>::DepthToUchar4(out, view->depth);

			break;
		case ITMKillingEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
			// use current raycast or forward projection?
			IITMVisualisationEngine::RenderRaycastSelection raycastType;
			if (trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
			else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

			// what sort of image is it?
			IITMVisualisationEngine::RenderImageType imageType;
			switch (getImageType) {
				case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
				case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case ITMKillingEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				default:
					imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
			}

			visualisationEngine->RenderImage(canonical_scene, trackingState->pose_d, &view->calib.intrinsics_d,
			                                 renderState_live, renderState_live->raycastImage, imageType, raycastType);

			ORUtils::Image<Vector4u>* srcImage = NULL;
			if (relocalisationCount != 0) srcImage = kfRaycast;
			else srcImage = renderState_live->raycastImage;

			out->ChangeDims(srcImage->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

			break;
		}
		case ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
			if (getImageType == ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
			else if (getImageType == ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			else if (getImageType == ITMKillingEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

			if (renderState_freeview == NULL) {
				renderState_freeview = ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims,
				                                                                        canonical_scene->sceneParams,
				                                                                        settings->GetMemoryType());
			}

			visualisationEngine->FindVisibleBlocks(canonical_scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->CreateExpectedDepths(canonical_scene, pose, intrinsics, renderState_freeview);
			visualisationEngine->RenderImage(canonical_scene, pose, intrinsics, renderState_freeview,
			                                 renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
			break;
		}
		case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
			break;
	};
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnTracking() { trackingActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffTracking() { trackingActive = false; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnIntegration() { fusionActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffIntegration() { fusionActive = false; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMKillingEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }
