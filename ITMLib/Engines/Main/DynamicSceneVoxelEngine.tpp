// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "DynamicSceneVoxelEngine.h"

#include "../LowLevel/LowLevelEngineFactory.h"
#include "../Meshing/MeshingEngineFactory.h"
#include "../ViewBuilding/ViewBuilderFactory.h"
#include "../Visualization/VisualizationEngineFactory.h"
#include "../VolumeFileIO/VolumeFileIOEngine.h"
#include "../../CameraTrackers/CameraTrackerFactory.h"

#include "../../../ORUtils/NVTimer.h"
#include "../../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

#include "../../../ORUtils/FileUtils.h"
#include "../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex>
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::DynamicSceneVoxelEngine(const RGBDCalib& calib, Vector2i imgSize_rgb,
                                                                        Vector2i imgSize_d) : settings(configuration::get()) {

	this->InitializeScenes();

	const MemoryDeviceType deviceType = settings.device_type;
	MemoryDeviceType memoryType = settings.device_type;
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;
	DynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().SetScenes(canonical_volume, live_volumes[0], warp_field);

	low_level_engine = LowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	view_builder = ViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	visualization_engine = VisualizationEngineFactory::MakeVisualizationEngine<TVoxel, TIndex>(deviceType);

	meshing_engine = nullptr;
	if (settings.create_meshing_engine)
		meshing_engine = MeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType, canonical_volume->index);

	denseMapper = new DenseDynamicMapper<TVoxel, TWarp, TIndex>(canonical_volume->index);
	Vector2i trackedImageSize = camera_tracking_controller->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	tracking_state = new CameraTrackingState(trackedImageSize, memoryType);


	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = CameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, low_level_engine, imuCalibrator,
	                                                canonical_volume->sceneParams);
	camera_tracking_controller = new CameraTrackingController(tracker);

	canonical_render_state = new RenderState(trackedImageSize, canonical_volume->sceneParams->near_clipping_distance,
	                                         canonical_volume->sceneParams->far_clipping_distance, settings.device_type);
	live_render_state = new RenderState(trackedImageSize, canonical_volume->sceneParams->near_clipping_distance,
	                                         canonical_volume->sceneParams->far_clipping_distance, settings.device_type);
	freeview_render_state = nullptr; //will be created if needed

	Reset();

	tracker->UpdateInitialPose(tracking_state);

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
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::InitializeScenes() {
	configuration::Configuration& settings = configuration::get();
	MemoryDeviceType memoryType = settings.device_type;
	this->canonical_volume = new VoxelVolume<TVoxel, TIndex>(
			&settings.general_voxel_volume_parameters, settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_CANONICAL));
	this->live_volumes = new VoxelVolume<TVoxel, TIndex>* [2];
	for (int iLiveScene = 0; iLiveScene < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_scene_count; iLiveScene++) {
		this->live_volumes[iLiveScene] = new VoxelVolume<TVoxel, TIndex>(
				&settings.general_voxel_volume_parameters,
				settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
				memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_LIVE));
	}
	this->warp_field = new VoxelVolume<TWarp, TIndex>(
			&settings.general_voxel_volume_parameters,
			settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_WARP));
}

template<typename TVoxel, typename TWarp, typename TIndex>
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::~DynamicSceneVoxelEngine() {
	delete canonical_render_state;
	delete live_render_state;
	delete freeview_render_state;
	if (freeview_render_state != nullptr) delete freeview_render_state;

	for (int iScene = 0; iScene < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_scene_count; iScene++) {
		delete live_volumes[iScene];
	}
	delete live_volumes;
	delete canonical_volume;

	delete denseMapper;
	delete camera_tracking_controller;

	delete tracker;
	delete imuCalibrator;

	delete low_level_engine;
	delete view_builder;

	delete tracking_state;
	delete view;

	delete visualization_engine;

	delete relocaliser;
	delete kfRaycast;

	delete meshing_engine;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::SaveSceneToMesh(const char* objFileName) {
	if (meshing_engine == nullptr) return;
	Mesh* mesh = new Mesh(configuration::get().device_type, canonical_volume->index.GetMaxVoxelCount());
	meshing_engine->MeshScene(mesh, canonical_volume);
	mesh->WriteSTL(objFileName);
	delete mesh;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::SaveToFile() {
	std::string nextFrameOutputPath = DynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().GetOutputDirectory();
	// throws error if any of the saves fail
	if (relocaliser) relocaliser->SaveToDirectory(nextFrameOutputPath + "/Relocaliser/");
	VolumeFileIOEngine<TVoxel, TIndex>::SaveToDirectoryCompact(canonical_volume, nextFrameOutputPath + "/canonical");
	VolumeFileIOEngine<TVoxel, TIndex>::SaveToDirectoryCompact(live_volumes[0], nextFrameOutputPath + "/live");
	std::cout << "Saving scenes in a compact way to '" << nextFrameOutputPath << "'." << std::endl;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::LoadFromFile() {
	std::string nextFrameOutputPath = DynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().GetOutputDirectory();
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
		VolumeFileIOEngine<TVoxel, TIndex>::LoadFromDirectoryCompact(canonical_volume,
		                                                               nextFrameOutputPath + "/canonical");
		VolumeFileIOEngine<TVoxel, TIndex>::LoadFromDirectoryCompact(live_volumes[0],
		                                                               nextFrameOutputPath + "/live");
		if (framesProcessed == 0) {
			framesProcessed = 1; //to skip initialization
		}
	}
	catch (std::runtime_error& e) {
		canonical_volume->Reset();
		throw std::runtime_error("Could not load scene:" + std::string(e.what()));
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::resetAll() {
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
CameraTrackingState::TrackingResult
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::ProcessFrame(ITMUChar4Image* rgbImage,
                                                             ITMShortImage* rawDepthImage,
                                                             IMUMeasurement* imuMeasurement) {

	// prepare image and turn it into a "view"
	if (imuMeasurement == nullptr)
		view_builder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                         settings.use_bilateral_filter, false, true);
	else
		view_builder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                         settings.use_bilateral_filter, imuMeasurement, false, true);

	if (!mainProcessingActive) {
		return CameraTrackingState::TRACKING_FAILED;
	}

	// camera tracking
	previousFramePose = (*(tracking_state->pose_d));
	if (trackingActive) camera_tracking_controller->Track(tracking_state, view);

	HandlePotentialCameraTrackingFailure();

	// surface tracking & fusion
	if (!mainProcessingActive) return CameraTrackingState::TRACKING_FAILED;
	fusion_succeeded = false;
	if ((last_tracking_result == CameraTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		if (framesProcessed > 0) {
			denseMapper->ProcessFrame(view, tracking_state, canonical_volume, live_volumes, warp_field, canonical_render_state);
		} else {
			denseMapper->ProcessInitialFrame(view, tracking_state, canonical_volume, live_volumes[0], canonical_render_state);
		}
		fusion_succeeded = true;
		if (framesProcessed > 50) trackingInitialised = true;
		framesProcessed++;
	}

	//preparation for next-frame tracking
	if (last_tracking_result == CameraTrackingState::TRACKING_GOOD ||
	    last_tracking_result == CameraTrackingState::TRACKING_POOR) {
		if (!fusion_succeeded) denseMapper->UpdateVisibleList(view, tracking_state, canonical_volume, canonical_render_state);

		// raycast to renderState_canonical for tracking and free Visualization
		camera_tracking_controller->Prepare(tracking_state, canonical_volume, view, visualization_engine, canonical_render_state);
	} else *tracking_state->pose_d = previousFramePose;

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

	return last_tracking_result;
}

template<typename TVoxel, typename TWarp, typename TIndex>
Vector2i DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::GetImageSize(void) const {
	return canonical_render_state->raycastImage->noDims;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::GetImage(ITMUChar4Image* out, GetImageType getImageType,
                                                              ORUtils::SE3Pose* pose,
                                                              Intrinsics* intrinsics) {
	auto& settings = configuration::get();
	if (view == nullptr) return;

	out->Clear();

	switch (getImageType) {
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
			break;
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA) view->depth->UpdateHostFromDevice();
			VisualizationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);
			break;
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
			// use current raycast or forward projection?
			IVisualizationEngine::RenderRaycastSelection raycastType;
			if (tracking_state->point_cloud_age <= 0) raycastType = IVisualizationEngine::RENDER_FROM_OLD_RAYCAST;
			else raycastType = IVisualizationEngine::RENDER_FROM_OLD_FORWARDPROJ;

			// what sort of image is it?
			IVisualizationEngine::RenderImageType imageType;
			switch (getImageType) {
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
					imageType = IVisualizationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
					imageType = IVisualizationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
					imageType = IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				default:
					imageType = IVisualizationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
			}

			visualization_engine->RenderImage(live_volumes[0], tracking_state->pose_d, &view->calib.intrinsics_d,
			                                  live_render_state, live_render_state->raycastImage, imageType,
			                                  raycastType);


			ORUtils::Image<Vector4u>* srcImage = nullptr;
			if (relocalisationCount != 0) srcImage = kfRaycast;
			else srcImage = canonical_render_state->raycastImage;

			out->ChangeDims(srcImage->noDims);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(srcImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(srcImage, MemoryCopyDirection::CPU_TO_CPU);

			break;
		}
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE;
			if (getImageType == DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)
				type = IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME;
			else if (getImageType == DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)
				type = IVisualizationEngine::RENDER_COLOUR_FROM_NORMAL;
			else if (getImageType == DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE)
				type = IVisualizationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

			if (freeview_render_state == nullptr) {
				freeview_render_state = new RenderState(out->noDims,
				                                        live_volumes[0]->sceneParams->near_clipping_distance,
				                                        live_volumes[0]->sceneParams->far_clipping_distance,
				                                        settings.device_type);
			}

			visualization_engine->FindVisibleBlocks(live_volumes[0], pose, intrinsics, freeview_render_state);
			visualization_engine->CreateExpectedDepths(live_volumes[0], pose, intrinsics, freeview_render_state);
			visualization_engine->RenderImage(live_volumes[0], pose, intrinsics, freeview_render_state, freeview_render_state->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(freeview_render_state->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(freeview_render_state->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}
		case MainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL: {
			IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE;

			if (freeview_render_state == nullptr) {
				freeview_render_state = new RenderState(out->noDims,
				                                        canonical_volume->sceneParams->near_clipping_distance,
				                                        canonical_volume->sceneParams->far_clipping_distance,
				                                        settings.device_type);
			}

			visualization_engine->FindVisibleBlocks(canonical_volume, pose, intrinsics, freeview_render_state);
			visualization_engine->CreateExpectedDepths(canonical_volume, pose, intrinsics, freeview_render_state);
			visualization_engine->RenderImage(canonical_volume, pose, intrinsics, freeview_render_state,
			                                  freeview_render_state->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(freeview_render_state->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(freeview_render_state->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}

		case MainEngine::InfiniTAM_IMAGE_UNKNOWN:
			break;
	};
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOnTracking() { trackingActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOffTracking() { trackingActive = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOnIntegration() { fusionActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOffIntegration() { fusionActive = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }

// region ==================================== STEP-BY-STEP MODE =======================================================


template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::HandlePotentialCameraTrackingFailure() {

	auto& settings = configuration::get();
	last_tracking_result = CameraTrackingState::TRACKING_GOOD;
	switch (settings.behavior_on_failure) {
		case configuration::FAILUREMODE_RELOCALIZE:
			//relocalisation
			last_tracking_result = tracking_state->trackerResult;
			if (last_tracking_result == CameraTrackingState::TRACKING_GOOD && relocalisationCount > 0)
				relocalisationCount--;

			view->depth->UpdateHostFromDevice();

			{
				int NN;
				float distances;
				//find and add keyframe, if necessary
				bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, tracking_state->pose_d, 0, 1, &NN,
				                                                  &distances,
				                                                  last_tracking_result ==
				                                                  CameraTrackingState::TRACKING_GOOD &&
				                                                  relocalisationCount == 0);

				//frame not added and tracking failed -> we need to relocalise
				if (!hasAddedKeyframe && last_tracking_result == CameraTrackingState::TRACKING_FAILED) {
					relocalisationCount = 10;

					// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
					view->rgb_prev->Clear();

					const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocaliser->RetrievePose(NN);
					tracking_state->pose_d->SetFrom(&keyframe.pose);

					denseMapper->UpdateVisibleList(view, tracking_state, live_volumes[0], canonical_render_state, true);

					camera_tracking_controller->Prepare(tracking_state, live_volumes[0], view, visualization_engine, canonical_render_state);
					camera_tracking_controller->Track(tracking_state, view);

					last_tracking_result = tracking_state->trackerResult;
				}
			}
			break;
		case configuration::FAILUREMODE_STOP_INTEGRATION:
			if (tracking_state->trackerResult != CameraTrackingState::TRACKING_FAILED)
				last_tracking_result = tracking_state->trackerResult;
			else last_tracking_result = CameraTrackingState::TRACKING_POOR;
			break;
		default:
			break;
	}


}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::Reset() {
	canonical_volume->Reset();
	for (int iScene = 0; iScene < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_scene_count; iScene++) {
		live_volumes[iScene]->Reset();
	}
	warp_field->Reset();
	tracking_state->Reset();
}

// endregion ===========================================================================================================