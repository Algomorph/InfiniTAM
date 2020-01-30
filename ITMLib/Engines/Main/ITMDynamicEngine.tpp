// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDynamicEngine.h"

#include "../LowLevel/ITMLowLevelEngineFactory.h"
#include "../Meshing/ITMMeshingEngineFactory.h"
#include "../ViewBuilding/ITMViewBuilderFactory.h"
#include "../Visualization/ITMVisualizationEngineFactory.h"
#include "../SceneFileIO/ITMSceneFileIOEngine.h"
#include "../../CameraTrackers/ITMCameraTrackerFactory.h"

#include "../../../ORUtils/NVTimer.h"
#include "../../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

#include "../../../ORUtils/FileUtils.h"
#include "../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex>
ITMDynamicEngine<TVoxel, TWarp, TIndex>::ITMDynamicEngine(const ITMRGBDCalib& calib, Vector2i imgSize_rgb,
                                                          Vector2i imgSize_d) {

	this->InitializeScenes();
	configuration::Configuration& settings = configuration::get();
	const MemoryDeviceType deviceType = settings.device_type;
	MemoryDeviceType memoryType = settings.device_type;
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;
	ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().SetScenes(canonicalScene, liveScenes[0], warpField);

	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	liveVisualisationEngine = ITMVisualizationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);
	canonicalVisualisationEngine =
			ITMVisualizationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);

	meshingEngine = nullptr;
	if (settings.create_meshing_engine)
		meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType, canonicalScene->index);

	denseMapper = new DenseDynamicMapper<TVoxel, TWarp, TIndex>(canonicalScene->index);
	Vector2i trackedImageSize = cameraTrackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	trackingState = new ITMTrackingState(trackedImageSize, memoryType);


	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMCameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, lowLevelEngine, imuCalibrator,
	                                                   canonicalScene->sceneParams);
	cameraTrackingController = new ITMTrackingController(tracker);

	renderState_live = new ITMRenderState(trackedImageSize, canonicalScene->sceneParams->near_clipping_distance,
	                                      canonicalScene->sceneParams->far_clipping_distance, settings.device_type);
	renderState_freeview = nullptr; //will be created if needed

	Reset();

	tracker->UpdateInitialPose(trackingState);

	view = nullptr; // will be allocated by the view builder

	if (settings.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d,
		                                                   Vector2f(
				                                                   settings.general_voxel_volume_parameters.near_clipping_distance,
				                                                   settings.general_voxel_volume_parameters.far_clipping_distance),
		                                                   0.2f, 500, 4);
	else relocaliser = nullptr;

	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::InitializeScenes() {
	configuration::Configuration& settings = configuration::get();
	MemoryDeviceType memoryType = settings.device_type;
	this->canonicalScene = new ITMVoxelVolume<TVoxel, TIndex>(
			&settings.general_voxel_volume_parameters, settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_CANONICAL));
	this->liveScenes = new ITMVoxelVolume<TVoxel, TIndex>* [2];
	for (int iLiveScene = 0; iLiveScene < ITMDynamicEngine<TVoxel, TWarp, TIndex>::liveSceneCount; iLiveScene++) {
		this->liveScenes[iLiveScene] = new ITMVoxelVolume<TVoxel, TIndex>(
				&settings.general_voxel_volume_parameters,
				settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
				memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_LIVE));
	}
	this->warpField = new ITMVoxelVolume<TWarp, TIndex>(
			&settings.general_voxel_volume_parameters,
			settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_WARP));
}

template<typename TVoxel, typename TWarp, typename TIndex>
ITMDynamicEngine<TVoxel, TWarp, TIndex>::~ITMDynamicEngine() {
	delete renderState_live;
	if (renderState_freeview != nullptr) delete renderState_freeview;

	for (int iScene = 0; iScene < ITMDynamicEngine<TVoxel, TWarp, TIndex>::liveSceneCount; iScene++) {
		delete liveScenes[iScene];
	}
	delete liveScenes;
	delete canonicalScene;

	delete denseMapper;
	delete cameraTrackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != nullptr) delete view;

	delete liveVisualisationEngine;
	delete canonicalVisualisationEngine;

	if (relocaliser != nullptr) delete relocaliser;
	delete kfRaycast;

	if (meshingEngine != nullptr) delete meshingEngine;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::SaveSceneToMesh(const char* objFileName) {
	if (meshingEngine == nullptr) return;
	ITMMesh* mesh = new ITMMesh(configuration::get().device_type, canonicalScene->index.GetMaxVoxelCount());
	meshingEngine->MeshScene(mesh, canonicalScene);
	mesh->WriteSTL(objFileName);
	delete mesh;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::SaveToFile() {
	std::string nextFrameOutputPath = ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().GetOutputDirectory();
	// throws error if any of the saves fail
	if (relocaliser) relocaliser->SaveToDirectory(nextFrameOutputPath + "/Relocaliser/");
	ITMSceneFileIOEngine<TVoxel, TIndex>::SaveToDirectoryCompact(canonicalScene, nextFrameOutputPath + "/canonical");
	ITMSceneFileIOEngine<TVoxel, TIndex>::SaveToDirectoryCompact(liveScenes[0], nextFrameOutputPath + "/live");
	std::cout << "Saving scenes in a compact way to '" << nextFrameOutputPath << "'." << std::endl;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::LoadFromFile() {
	std::string nextFrameOutputPath = ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().GetOutputDirectory();
	std::string relocaliserInputDirectory = nextFrameOutputPath + "Relocaliser/";

	////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
	////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

	this->resetAll();

	if (view != nullptr) {
		try // load relocaliser
		{
			auto& settings = configuration::get();
			FernRelocLib::Relocaliser<float>* relocaliser_temp =
					new FernRelocLib::Relocaliser<float>(view->depth->noDims,
					                                     Vector2f(
							                                     settings.general_voxel_volume_parameters.near_clipping_distance,
							                                     settings.general_voxel_volume_parameters.far_clipping_distance),
					                                     0.2f, 500, 4);

			relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

			delete relocaliser;
			relocaliser = relocaliser_temp;
		}
		catch (std::runtime_error& e) {
			throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
		}
	}

	try // load scene
	{
		std::cout << "Loading scenes from '" << nextFrameOutputPath << "'." << std::endl;
		ITMSceneFileIOEngine<TVoxel, TIndex>::LoadFromDirectoryCompact(canonicalScene,
		                                                               nextFrameOutputPath + "/canonical");
		ITMSceneFileIOEngine<TVoxel, TIndex>::LoadFromDirectoryCompact(liveScenes[0],
		                                                               nextFrameOutputPath + "/live");
		if (framesProcessed == 0) {
			framesProcessed = 1; //to skip initialization
		}
	}
	catch (std::runtime_error& e) {
		canonicalScene->Reset();
		throw std::runtime_error("Could not load scene:" + std::string(e.what()));
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::resetAll() {
	Reset();
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


template<typename TVoxel, typename TWarp, typename TIndex>
ITMTrackingState::TrackingResult
ITMDynamicEngine<TVoxel, TWarp, TIndex>::ProcessFrame(ITMUChar4Image* rgbImage,
                                                      ITMShortImage* rawDepthImage,
                                                      ITMIMUMeasurement* imuMeasurement) {

	BeginProcessingFrame(rgbImage, rawDepthImage, imuMeasurement);
	if (!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;
	fusionSucceeded = false;
	if ((lastTrackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		if (framesProcessed > 0) {
			denseMapper->ProcessFrame(view, trackingState, canonicalScene, liveScenes, warpField, renderState_live);
		} else {
			denseMapper->ProcessInitialFrame(view, trackingState, canonicalScene, liveScenes[0], renderState_live);
		}
		fusionSucceeded = true;
		if (framesProcessed > 50) trackingInitialised = true;
		framesProcessed++;
	}

	if (lastTrackerResult == ITMTrackingState::TRACKING_GOOD ||
	    lastTrackerResult == ITMTrackingState::TRACKING_POOR) {
		if (!fusionSucceeded) denseMapper->UpdateVisibleList(view, trackingState, liveScenes[0], renderState_live);

		// raycast to renderState_live for tracking and free visualisation
		cameraTrackingController->Prepare(trackingState, liveScenes[0], view, liveVisualisationEngine,
		                                  renderState_live);
	} else *trackingState->pose_d = previousFramePose;

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

	return lastTrackerResult;
}

template<typename TVoxel, typename TWarp, typename TIndex>
Vector2i ITMDynamicEngine<TVoxel, TWarp, TIndex>::GetImageSize(void) const {
	return renderState_live->raycastImage->noDims;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::GetImage(ITMUChar4Image* out, GetImageType getImageType,
                                                       ORUtils::SE3Pose* pose,
                                                       ITMIntrinsics* intrinsics) {
	auto& settings = configuration::get();
	if (view == nullptr) return;

	out->Clear();

	switch (getImageType) {
		case ITMDynamicEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
			break;
		case ITMDynamicEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);
			break;
		case ITMDynamicEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
			// use current raycast or forward projection?
			IITMVisualisationEngine::RenderRaycastSelection raycastType;
			if (trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
			else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

			// what sort of image is it?
			IITMVisualisationEngine::RenderImageType imageType;
			switch (getImageType) {
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				default:
					imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
			}

			liveVisualisationEngine->RenderImage(liveScenes[0], trackingState->pose_d, &view->calib.intrinsics_d,
			                                     renderState_live, renderState_live->raycastImage, imageType,
			                                     raycastType);


			ORUtils::Image<Vector4u>* srcImage = nullptr;
			if (relocalisationCount != 0) srcImage = kfRaycast;
			else srcImage = renderState_live->raycastImage;

			out->ChangeDims(srcImage->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(srcImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(srcImage, MemoryCopyDirection::CPU_TO_CPU);

			break;
		}
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
			if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
			else if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			else if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

			if (renderState_freeview == nullptr) {
				renderState_freeview = new ITMRenderState(out->noDims,
				                                          liveScenes[0]->sceneParams->near_clipping_distance,
				                                          liveScenes[0]->sceneParams->far_clipping_distance,
				                                          settings.device_type);
			}

			liveVisualisationEngine->FindVisibleBlocks(liveScenes[0], pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->CreateExpectedDepths(liveScenes[0], pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->RenderImage(liveScenes[0], pose, intrinsics, renderState_freeview,
			                                     renderState_freeview->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}
		case ITMMainEngine::InfiniTAM_IMAGE_STEP_BY_STEP: {

			if (renderState_freeview == nullptr) {
				renderState_freeview = new ITMRenderState(out->noDims,
				                                          liveScenes[0]->sceneParams->near_clipping_distance,
				                                          liveScenes[0]->sceneParams->far_clipping_distance,
				                                          settings.device_type);
			}

			liveVisualisationEngine->FindVisibleBlocks(liveScenes[0], pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->CreateExpectedDepths(liveScenes[0], pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->RenderImage(liveScenes[0], pose, intrinsics, renderState_freeview,
			                                     renderState_freeview->raycastImage,
			                                     IITMVisualisationEngine::RENDER_SHADED_GREEN);
			canonicalVisualisationEngine->FindVisibleBlocks(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->CreateExpectedDepths(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->RenderImage(canonicalScene, pose, intrinsics, renderState_freeview,
			                                          renderState_freeview->raycastImage,
			                                          IITMVisualisationEngine::RENDER_SHADED_OVERLAY);


			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CPU_TO_CPU);


			break;
		}
		case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL: {
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

			if (renderState_freeview == nullptr) {
				renderState_freeview = new ITMRenderState(out->noDims,
				                                          canonicalScene->sceneParams->near_clipping_distance,
				                                          canonicalScene->sceneParams->far_clipping_distance,
				                                          settings.device_type);
			}

			canonicalVisualisationEngine->FindVisibleBlocks(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->CreateExpectedDepths(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->RenderImage(canonicalScene, pose, intrinsics, renderState_freeview,
			                                          renderState_freeview->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}

		case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
			break;
	};
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOnTracking() { trackingActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOffTracking() { trackingActive = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOnIntegration() { fusionActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOffIntegration() { fusionActive = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }

// region ==================================== STEP-BY-STEP MODE =======================================================


template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::BeginProcessingFrame(ITMUChar4Image* rgbImage,
                                                                   ITMShortImage* rawDepthImage,
                                                                   ITMIMUMeasurement* imuMeasurement) {

	auto& settings = configuration::get();

	// prepare image and turn it into a depth image
	if (imuMeasurement == nullptr)
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, false, true);
	else
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, imuMeasurement, false, true);

	if (!mainProcessingActive) {
		lastTrackerResult = ITMTrackingState::TRACKING_FAILED;
		return;
	}

	// tracking
	previousFramePose = (*(trackingState->pose_d));
	if (trackingActive) cameraTrackingController->Track(trackingState, view);

	lastTrackerResult = ITMTrackingState::TRACKING_GOOD;

	switch (settings.behavior_on_failure) {
		case configuration::FAILUREMODE_RELOCALIZE:
			//relocalisation
			lastTrackerResult = trackingState->trackerResult;
			if (lastTrackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0)
				relocalisationCount--;

			view->depth->UpdateHostFromDevice();

			{
				int NN;
				float distances;
				//find and add keyframe, if necessary
				bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN,
				                                                  &distances,
				                                                  lastTrackerResult ==
				                                                  ITMTrackingState::TRACKING_GOOD &&
				                                                  relocalisationCount == 0);

				//frame not added and tracking failed -> we need to relocalise
				if (!hasAddedKeyframe && lastTrackerResult == ITMTrackingState::TRACKING_FAILED) {
					relocalisationCount = 10;

					// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
					view->rgb_prev->Clear();

					const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocaliser->RetrievePose(NN);
					trackingState->pose_d->SetFrom(&keyframe.pose);

					denseMapper->UpdateVisibleList(view, trackingState, liveScenes[0], renderState_live, true);

					cameraTrackingController->Prepare(trackingState, liveScenes[0], view, liveVisualisationEngine,
					                                  renderState_live);
					cameraTrackingController->Track(trackingState, view);

					lastTrackerResult = trackingState->trackerResult;
				}
			}
			break;
		case configuration::FAILUREMODE_STOP_INTEGRATION:
			if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
				lastTrackerResult = trackingState->trackerResult;
			else lastTrackerResult = ITMTrackingState::TRACKING_POOR;
			break;
		default:
			break;
	}


}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMDynamicEngine<TVoxel, TWarp, TIndex>::Reset() {
	canonicalScene->Reset();
	for (int iScene = 0; iScene < ITMDynamicEngine<TVoxel, TWarp, TIndex>::liveSceneCount; iScene++) {
		liveScenes[iScene]->Reset();
	}
	warpField->Reset();
	trackingState->Reset();
}

// endregion ===========================================================================================================