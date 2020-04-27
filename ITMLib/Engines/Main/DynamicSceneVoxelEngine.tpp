//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2000 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

//stdlib
#include <unordered_set>
#include <vector>

//log4cplus
#include <log4cplus/loggingmacros.h>

// local
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
#include "../../Utils/Logging/LoggingConfigruation.h"
#include "../Analytics/AnalyticsEngineFactory.h"
#include "../../Utils/Analytics/BenchmarkUtilities.h"
#include "../VolumeFusion/VolumeFusionEngineFactory.h"
#include "../DepthFusion/DepthFusionEngineFactory.h"
#include "../Indexing/IndexingEngineFactory.h"
#include "../../SurfaceTrackers/SurfaceTrackerFactory.h"
#include "../Swapping/SwappingEngineFactory.h"
#include "../Analytics/AnalyticsLogging.h"
#include "../Analytics/AnalyticsTelemetry.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex>
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::DynamicSceneVoxelEngine(const RGBDCalib& calibration_info,
                                                                        Vector2i rgb_image_size,
                                                                        Vector2i depth_image_size)
		: config(configuration::get()),
		  indexing_engine(IndexingEngineFactory::Build<TVoxel, TIndex>(configuration::get().device_type)),
		  depth_fusion_engine(
				  DepthFusionEngineFactory::Build<TVoxel, TWarp, TIndex>
						  (configuration::get().device_type)),
		  volume_fusion_engine(VolumeFusionEngineFactory::Build<TVoxel, TIndex>(configuration::get().device_type)),
		  surface_tracker(SurfaceTrackerFactory::Build<TVoxel, TWarp, TIndex>()),
		  swapping_engine(configuration::get().swapping_mode != configuration::SWAPPINGMODE_DISABLED ?
		                  SwappingEngineFactory::Build<TVoxel, TIndex>(
				                  configuration::get().device_type,
				                  configuration::for_volume_role<TIndex>(configuration::VOLUME_CANONICAL)
		                  ) : nullptr),
		  low_level_engine(LowLevelEngineFactory::MakeLowLevelEngine(configuration::get().device_type)),
		  view_builder(ViewBuilderFactory::Build(calibration_info, configuration::get().device_type)),
		  visualization_engine(VisualizationEngineFactory::MakeVisualizationEngine<TVoxel, TIndex>(
				  configuration::get().device_type)),
		  meshing_engine(config.create_meshing_engine ? MeshingEngineFactory::Build<TVoxel, TIndex>(
		  		configuration::get().device_type) : nullptr) {
	logging::initialize_logging();

	this->InitializeScenes();

	camera_tracking_enabled = config.enable_rigid_tracking;

	const MemoryDeviceType device_type = config.device_type;
	MemoryDeviceType memoryType = config.device_type;
	if ((depth_image_size.x == -1) || (depth_image_size.y == -1)) depth_image_size = rgb_image_size;

	imu_calibrator = new ITMIMUCalibrator_iPad();
	camera_tracker = CameraTrackerFactory::Instance().Make(rgb_image_size, depth_image_size, low_level_engine,
	                                                       imu_calibrator,
	                                                       canonical_volume->GetParameters());
	camera_tracking_controller = new CameraTrackingController(camera_tracker);
	//TODO: is "tracked" image size ever different from actual depth image size? If so, document GetTrackedImageSize function. Otherwise, revise.
	Vector2i tracked_image_size = camera_tracking_controller->GetTrackedImageSize(rgb_image_size, depth_image_size);
	tracking_state = new CameraTrackingState(tracked_image_size, memoryType);


	canonical_render_state = new RenderState(tracked_image_size,
	                                         canonical_volume->GetParameters().near_clipping_distance,
	                                         canonical_volume->GetParameters().far_clipping_distance,
	                                         config.device_type);
	live_render_state = new RenderState(tracked_image_size, canonical_volume->GetParameters().near_clipping_distance,
	                                    canonical_volume->GetParameters().far_clipping_distance,
	                                    config.device_type);
	freeview_render_state = nullptr; //will be created if needed



	Reset();

	camera_tracker->UpdateInitialPose(tracking_state);

	view = nullptr; // will be allocated by the view builder

	if (config.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE)
		relocalizer = new FernRelocLib::Relocaliser<float>(depth_image_size,
		                                                   Vector2f(
				                                                   config.general_voxel_volume_parameters.near_clipping_distance,
				                                                   config.general_voxel_volume_parameters.far_clipping_distance),
		                                                   0.2f, 500, 4);
	else relocalizer = nullptr;

	keyframe_raycast = new ITMUChar4Image(depth_image_size, memoryType);

	fusion_active = true;
	main_processing_active = true;
	tracking_initialised = false;
	relocalization_count = 0;
	frames_processed = 0;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::InitializeScenes() {
	configuration::Configuration& settings = configuration::get();
	MemoryDeviceType memoryType = settings.device_type;
	this->canonical_volume = new VoxelVolume<TVoxel, TIndex>(
			settings.general_voxel_volume_parameters, settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_CANONICAL));
	this->live_volumes = new VoxelVolume<TVoxel, TIndex>* [2];
	for (int iLiveScene = 0;
	     iLiveScene < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_volume_count; iLiveScene++) {
		this->live_volumes[iLiveScene] = new VoxelVolume<TVoxel, TIndex>(
				settings.general_voxel_volume_parameters,
				settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
				memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_LIVE));
	}
	this->warp_field = new VoxelVolume<TWarp, TIndex>(
			settings.general_voxel_volume_parameters,
			settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryType, configuration::for_volume_role<TIndex>(configuration::VOLUME_WARP));
}

template<typename TVoxel, typename TWarp, typename TIndex>
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::~DynamicSceneVoxelEngine() {

	delete visualization_engine;
	delete meshing_engine;
	delete depth_fusion_engine;
	delete swapping_engine;
	delete low_level_engine;
	delete view_builder;

	delete camera_tracking_controller;

	delete surface_tracker;
	delete camera_tracker;
	delete imu_calibrator;

	for (int i_volume = 0; i_volume < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_volume_count; i_volume++) {
		delete live_volumes[i_volume];
	}
	delete live_volumes;
	delete canonical_volume;

	delete canonical_render_state;
	delete live_render_state;
	if (freeview_render_state != nullptr) delete freeview_render_state;

	delete keyframe_raycast;
	delete relocalizer;

	delete view;
	delete tracking_state;

	delete canonical_volume_memory_usage_file;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::SaveSceneToMesh(const char* path) {
	if (meshing_engine == nullptr) return;
	Mesh mesh = meshing_engine->MeshScene(canonical_volume);
	mesh.WriteSTL(path);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::SaveToFile(const std::string& path) {
	std::string relocalizer_output_path = path + "Relocaliser/";
	// throws error if any of the saves fail
	if (relocalizer) relocalizer->SaveToDirectory(relocalizer_output_path);
	canonical_volume->SaveToDisk(path + "/canonical_volume.dat");
	live_volumes[0]->SaveToDisk(path + "/live_volume.dat");
	std::cout << "Saving scenes in a compact way to '" << path << "'." << std::endl;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::SaveToFile() {
	fs::path path(std::string(config.paths.output_path) + "/ProcessedFrame_" + std::to_string(frames_processed));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	SaveToFile(path.string());
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::LoadFromFile(const std::string& path) {
	std::string relocaliser_input_path = path + "Relocaliser/";
	if (view != nullptr) {
		try // load relocaliser
		{
			auto& settings = configuration::get();
			FernRelocLib::Relocaliser<float>* relocaliser_temp =
					new FernRelocLib::Relocaliser<float>(view->depth->dimensions,
					                                     Vector2f(
							                                     settings.general_voxel_volume_parameters.near_clipping_distance,
							                                     settings.general_voxel_volume_parameters.far_clipping_distance),
					                                     0.2f, 500, 4);

			relocaliser_temp->LoadFromDirectory(relocaliser_input_path);

			delete relocalizer;
			relocalizer = relocaliser_temp;
		}
		catch (std::runtime_error& e) {
			throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
		}
	}

	try // load scene
	{
		std::cout << "Loading canonical volume from '" << path << "'." << std::endl;
		canonical_volume->LoadFromDisk(path + "/canonical");

		if (frames_processed == 0) {
			frames_processed = 1; //to skip initialization
		}
	}
	catch (std::runtime_error& e) {
		canonical_volume->Reset();
		throw std::runtime_error("Could not load volume:" + std::string(e.what()));
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::LoadFromFile() {
	fs::path path(std::string(config.paths.output_path) + "/ProcessedFrame_" + std::to_string(frames_processed));
	LoadFromFile(path.string());
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::ResetAll() {
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
DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::ProcessFrame(ITMUChar4Image* rgb_image,
                                                             ITMShortImage* depth_image,
                                                             IMUMeasurement* imu_measurement) {
	// prepare images & IMU measurements and turn them into a "view"
	if (imu_measurement == nullptr) {
		view_builder->UpdateView(&view, rgb_image, depth_image, config.use_threshold_filter,
		                         config.use_bilateral_filter, false, true);
	} else {
		view_builder->UpdateView(&view, rgb_image, depth_image, config.use_threshold_filter,
		                         config.use_bilateral_filter, imu_measurement, false, true);
	}

	if (!main_processing_active) {
		return CameraTrackingState::TRACKING_FAILED;
	}
	// camera tracking
	previous_frame_pose = (*(tracking_state->pose_d));
	if (camera_tracking_enabled) camera_tracking_controller->Track(tracking_state, view);

	HandlePotentialCameraTrackingFailure();

	// surface tracking & fusion
	if (!main_processing_active) return CameraTrackingState::TRACKING_FAILED;
	bool fusion_succeeded = false;
	if ((last_tracking_result == CameraTrackingState::TRACKING_GOOD || !tracking_initialised) && (fusion_active) &&
	    (relocalization_count == 0)) {
		if (frames_processed > 0) {
			LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan << "Generating raw live TSDF from view..." << reset);
			benchmarking::start_timer("GenerateRawLiveVolume");
			live_volumes[0]->Reset();
			live_volumes[1]->Reset();
			warp_field->Reset();

			indexing_engine->AllocateNearAndBetweenTwoSurfaces(live_volumes[0], view, tracking_state);
			indexing_engine->AllocateFromOtherVolume(live_volumes[1], live_volumes[0]);
			indexing_engine->AllocateFromOtherVolume(canonical_volume, live_volumes[0]);
			indexing_engine->AllocateWarpVolumeFromOtherVolume(warp_field, live_volumes[0]);
			depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(live_volumes[0], view, tracking_state);


			LogTSDFVolumeStatistics(live_volumes[0], "[[live TSDF before tracking]]");
			benchmarking::stop_timer("GenerateRawLiveVolume");

			benchmarking::start_timer("TrackMotion");
			LOG4CPLUS_PER_FRAME(logging::get_logger(), bright_cyan
					<< "*** Optimizing warp based on difference between canonical and live SDF. ***" << reset);
			VoxelVolume<TVoxel, TIndex>* target_warped_live_volume =
					surface_tracker->TrackFrameMotion(canonical_volume, live_volumes, warp_field);
			LOG4CPLUS_PER_FRAME(logging::get_logger(),
			                    bright_cyan << "*** Warping optimization finished for current frame. ***" << reset);
			benchmarking::stop_timer("TrackMotion");
			LogTSDFVolumeStatistics(target_warped_live_volume, "[[live TSDF after tracking]]");


			//fuse warped live into canonical
			benchmarking::start_timer("FuseOneTsdfVolumeIntoAnother");
			//FIXME: use proper frame number here
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, target_warped_live_volume, 0);
			benchmarking::stop_timer("FuseOneTsdfVolumeIntoAnother");

			LogTSDFVolumeStatistics(canonical_volume, "[[canonical TSDF after fusion]]");
			RecordVolumeMemoryUsageInfo(*canonical_volume_memory_usage_file, *canonical_volume);

		} else {
			LOG4CPLUS_PER_FRAME(logging::get_logger(),
			                    bright_cyan << "Generating raw live frame from view..." << reset);
			benchmarking::start_timer("GenerateRawLiveAndCanonicalVolumes");
			indexing_engine->AllocateNearSurface(live_volumes[0], view, tracking_state);
			depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(live_volumes[0], view, tracking_state);
			benchmarking::stop_timer("GenerateRawLiveAndCanonicalVolumes");
			//** prepare canonical for new frame
			LOG4CPLUS_PER_FRAME(logging::get_logger(),
			                    bright_cyan << "Fusing data from live frame into canonical frame..." << reset);
			//** fuse the live into canonical directly
			benchmarking::start_timer("FuseOneTsdfVolumeIntoAnother");
			indexing_engine->AllocateFromOtherVolume(canonical_volume, live_volumes[0]);
			//FIXME: use proper frame number here instead of 0
			volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, live_volumes[0], frames_processed);
			benchmarking::stop_timer("FuseOneTsdfVolumeIntoAnother");
		}
		fusion_succeeded = true;
		if (frames_processed > 50) tracking_initialised = true;
		frames_processed++;
	}

	//preparation for next-frame tracking
	if (last_tracking_result == CameraTrackingState::TRACKING_GOOD ||
	    last_tracking_result == CameraTrackingState::TRACKING_POOR) {
		//TODO: FIXME (See issue #214 at https://github.com/Algomorph/InfiniTAM/issues/214)
		//if (!fusion_succeeded) indexing_engine->UpdateVisibleList(view, tracking_state, canonical_volume, canonical_render_state);

		// raycast to renderState_canonical for tracking and free Visualization
		camera_tracking_controller->Prepare(tracking_state, canonical_volume, view, visualization_engine,
		                                    canonical_render_state);
	} else *tracking_state->pose_d = previous_frame_pose;

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
Vector2i DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::GetImageSize() const {
	return canonical_render_state->raycastImage->dimensions;
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::GetImage(ITMUChar4Image* out, GetImageType type,
                                                              ORUtils::SE3Pose* pose,
                                                              Intrinsics* intrinsics) {


	auto& settings = configuration::get();
	if (view == nullptr) return;

	out->Clear();

	switch (type) {
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb->dimensions);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*view->rgb, MemoryCopyDirection::CPU_TO_CPU);
			break;
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth->dimensions);
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
			IVisualizationEngine::RenderImageType render_type;
			switch (type) {
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				default:
					render_type = IVisualizationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
			}

			visualization_engine->RenderImage(live_volumes[0], tracking_state->pose_d, &view->calib.intrinsics_d,
			                                  live_render_state, live_render_state->raycastImage, render_type,
			                                  raycastType);


			ORUtils::Image<Vector4u>* srcImage = nullptr;
			if (relocalization_count != 0) srcImage = keyframe_raycast;
			else srcImage = canonical_render_state->raycastImage;

			out->ChangeDims(srcImage->dimensions);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*srcImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*srcImage, MemoryCopyDirection::CPU_TO_CPU);

			break;
		}
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			IVisualizationEngine::RenderImageType render_type = IVisualizationEngine::RENDER_SHADED_GREYSCALE;
			switch (type) {
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case DynamicSceneVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
					render_type = IVisualizationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
			}

			if (freeview_render_state == nullptr) {
				freeview_render_state = new RenderState(out->dimensions,
				                                        live_volumes[0]->GetParameters().near_clipping_distance,
				                                        live_volumes[0]->GetParameters().far_clipping_distance,
				                                        settings.device_type);
			}

			visualization_engine->FindVisibleBlocks(live_volumes[0], pose, intrinsics, freeview_render_state);
			visualization_engine->CreateExpectedDepths(live_volumes[0], pose, intrinsics, freeview_render_state);
			visualization_engine->RenderImage(live_volumes[0], pose, intrinsics, freeview_render_state,
			                                  freeview_render_state->raycastImage, render_type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*freeview_render_state->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*freeview_render_state->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}
		case MainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL: {
			IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE;

			if (freeview_render_state == nullptr) {
				freeview_render_state = new RenderState(out->dimensions,
				                                        canonical_volume->GetParameters().near_clipping_distance,
				                                        canonical_volume->GetParameters().far_clipping_distance,
				                                        settings.device_type);
			}

			visualization_engine->FindVisibleBlocks(canonical_volume, pose, intrinsics, freeview_render_state);
			visualization_engine->CreateExpectedDepths(canonical_volume, pose, intrinsics, freeview_render_state);
			visualization_engine->RenderImage(canonical_volume, pose, intrinsics, freeview_render_state,
			                                  freeview_render_state->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*freeview_render_state->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*freeview_render_state->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
			break;
		}

		case MainEngine::InfiniTAM_IMAGE_UNKNOWN:
			break;
	};
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOnTracking() { camera_tracking_enabled = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOffTracking() { camera_tracking_enabled = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOnIntegration() { fusion_active = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOffIntegration() { fusion_active = false; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOnMainProcessing() { main_processing_active = true; }

template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::TurnOffMainProcessing() { main_processing_active = false; }

// region ==================================== STEP-BY-STEP MODE =======================================================


template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::HandlePotentialCameraTrackingFailure() {

	auto& settings = configuration::get();
	last_tracking_result = CameraTrackingState::TRACKING_GOOD;
	switch (settings.behavior_on_failure) {
		case configuration::FAILUREMODE_RELOCALIZE:
			//relocalisation
			last_tracking_result = tracking_state->trackerResult;
			if (last_tracking_result == CameraTrackingState::TRACKING_GOOD && relocalization_count > 0)
				relocalization_count--;

			view->depth->UpdateHostFromDevice();

			{
				int NN;
				float distances;
				//find and add keyframe, if necessary
				bool hasAddedKeyframe = relocalizer->ProcessFrame(view->depth, tracking_state->pose_d, 0, 1, &NN,
				                                                  &distances,
				                                                  last_tracking_result ==
				                                                  CameraTrackingState::TRACKING_GOOD &&
				                                                  relocalization_count == 0);

				//frame not added and tracking failed -> we need to relocalise
				if (!hasAddedKeyframe && last_tracking_result == CameraTrackingState::TRACKING_FAILED) {
					relocalization_count = 10;

					// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
					view->rgb_prev->Clear();

					const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocalizer->RetrievePose(NN);
					tracking_state->pose_d->SetFrom(&keyframe.pose);

					//TODO: FIXME (See issue #214 at https://github.com/Algomorph/InfiniTAM/issues/214)
					//indexing_engine->UpdateVisibleList(view, tracking_state, live_volumes[0], canonical_render_state, true);

					camera_tracking_controller->Prepare(tracking_state, live_volumes[0], view, visualization_engine,
					                                    canonical_render_state);
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
	for (int iScene = 0; iScene < DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::live_volume_count; iScene++) {
		live_volumes[iScene]->Reset();
	}
	warp_field->Reset();
	tracking_state->Reset();
}



template<typename TVoxel, typename TWarp, typename TIndex>
void DynamicSceneVoxelEngine<TVoxel, TWarp, TIndex>::ProcessSwapping(RenderState* render_state) {

	// swapping: CUDA -> CPU
	switch (config.swapping_mode) {
		case configuration::SWAPPINGMODE_ENABLED:
			swapping_engine->IntegrateGlobalIntoLocal(canonical_volume, render_state);
			swapping_engine->SaveToGlobalMemory(canonical_volume, render_state);
			break;
		case configuration::SWAPPINGMODE_DELETE:
			swapping_engine->CleanLocalMemory(canonical_volume, render_state);
			break;
		case configuration::SWAPPINGMODE_DISABLED:
			break;
	}
}

// endregion ===========================================================================================================