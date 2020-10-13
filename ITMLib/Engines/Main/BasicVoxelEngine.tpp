#include <filesystem>
#include "BasicVoxelEngine.h"
#include "../ImageProcessing/ImageProcessingEngineFactory.h"
#include "../Meshing/MeshingEngineFactory.h"
#include "../ViewBuilder/ViewBuilderFactory.h"
#include "../Rendering/RenderingEngineFactory.h"
#include "../../CameraTrackers/CameraTrackerFactory.h"

#include "../../../ORUtils/NVTimer.h"
#include "../../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

using namespace ITMLib;

template <typename TVoxel, typename TIndex>
BasicVoxelEngine<TVoxel,TIndex>::BasicVoxelEngine(const RGBD_CalibrationInformation& calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
	: tracking_active(this->parameters.enable_rigid_alignment){
	auto& settings = configuration::Get();

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	MemoryDeviceType memoryType = settings.device_type;
	this->volume = new VoxelVolume<TVoxel,TIndex>(memoryType);

	const MemoryDeviceType deviceType = settings.device_type;

	image_processing_engine = ImageProcessingEngineFactory::Build(deviceType);
	viewBuilder = ViewBuilderFactory::Build(calib, deviceType);
	visualization_engine = RenderingEngineFactory::Build<TVoxel, TIndex>(deviceType);

	meshing_engine = nullptr;
	if (settings.create_meshing_engine)
		meshing_engine = MeshingEngineFactory::Build<TVoxel, TIndex>(deviceType);

	denseMapper = new DenseMapper<TVoxel, TIndex>();
	volume->Reset();

	imu_calibrator = new ITMIMUCalibrator_iPad();
	tracker = CameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, image_processing_engine, imu_calibrator,
	                                                volume->GetParameters());
	trackingController = new CameraTrackingController(tracker);

	Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	render_state =  new RenderState(imgSize_d, volume->GetParameters().near_clipping_distance, volume->GetParameters().far_clipping_distance, memoryType);
	render_state_freeview = nullptr; //will be created if needed

	trackingState = new CameraTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = nullptr; // will be allocated by the view builder
	
	if (settings.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings.general_voxel_volume_parameters.near_clipping_distance, settings.general_voxel_volume_parameters.far_clipping_distance), 0.2f, 500, 4);
	else relocaliser = nullptr;

	kfRaycast = new UChar4Image(imgSize_d, memoryType);

	tracking_active = true;
	fusion_active = true;
	main_processing_active = true;
	tracking_initialised = false;
	relocalization_count = 0;
	processed_frame_count = 0;
}

template <typename TVoxel, typename TIndex>
BasicVoxelEngine<TVoxel,TIndex>::~BasicVoxelEngine()
{
	delete render_state;
	if (render_state_freeview != nullptr) delete render_state_freeview;

	delete volume;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imu_calibrator;

	delete image_processing_engine;
	delete viewBuilder;

	delete trackingState;
	if (view != nullptr) delete view;

	delete visualization_engine;

	if (relocaliser != nullptr) delete relocaliser;
	delete kfRaycast;

	if (meshing_engine != nullptr) delete meshing_engine;
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::SaveVolumeToMesh(const std::string& path)
{
	if (meshing_engine == nullptr) return;
	Mesh mesh = meshing_engine->MeshVolume(volume);
	mesh.WritePLY(path);
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel, TIndex>::SaveToFile()
{
	// throws error if any of the saves fail

	std::string output_directory = "State/";
	std::string relocalizer_output_directory = output_directory + "Relocaliser/", sceneOutputDirectory = output_directory + "Volume/";
	
	std::filesystem::create_directories(output_directory);
	std::filesystem::create_directories(relocalizer_output_directory);
	std::filesystem::create_directories(sceneOutputDirectory.c_str());

	if (relocaliser) relocaliser->SaveToDirectory(relocalizer_output_directory);

	volume->SaveToDisk(sceneOutputDirectory);
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel, TIndex>::LoadFromFile()
{
	auto& settings = configuration::Get();
	std::string saveInputDirectory = "State/";
	std::string relocaliserInputDirectory = saveInputDirectory + "Relocaliser/", sceneInputDirectory = saveInputDirectory + "Volume/";

	////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
	////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

	this->ResetAll();

	try // load relocaliser
	{
		FernRelocLib::Relocaliser<float> *relocaliser_temp = new FernRelocLib::Relocaliser<float>(view->depth.dimensions, Vector2f(settings.general_voxel_volume_parameters.near_clipping_distance, settings.general_voxel_volume_parameters.far_clipping_distance), 0.2f, 500, 4);

		relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

		delete relocaliser; 
		relocaliser = relocaliser_temp;
	}
	catch (std::runtime_error &e)
	{
		throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
	}

	try // load scene
	{
		volume->LoadFromDisk(sceneInputDirectory);
	}
	catch (std::runtime_error &e)
	{
		volume->Reset();
		throw std::runtime_error("Could not load scene:" + std::string(e.what()));
	}
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::ResetAll()
{
	volume->Reset();
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

template <typename TVoxel, typename TIndex>
CameraTrackingState::TrackingResult BasicVoxelEngine<TVoxel,TIndex>::ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, IMUMeasurement *imuMeasurement)
{
	auto& settings = configuration::Get();
	// prepare image and turn it into a depth image
	if (imuMeasurement == nullptr)
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, false, true);
	else
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, imuMeasurement, false, true);

	if (!main_processing_active) return CameraTrackingState::TRACKING_FAILED;

	// tracking
	ORUtils::SE3Pose oldPose(*(trackingState->pose_d));
	if (tracking_active) trackingController->Track(trackingState, view);

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
	int addKeyframeIdx = -1;
	if (settings.behavior_on_failure == configuration::FAILUREMODE_RELOCALIZE)
	{
		if (trackerResult == CameraTrackingState::TRACKING_GOOD && relocalization_count > 0) relocalization_count--;

		int NN; float distances;
		view->depth.UpdateHostFromDevice();

		//find and add keyframe, if necessary
		bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances, trackerResult == CameraTrackingState::TRACKING_GOOD && relocalization_count == 0);

		//frame not added and tracking failed -> we need to relocalise
		if (!hasAddedKeyframe && trackerResult == CameraTrackingState::TRACKING_FAILED)
		{
			relocalization_count = 10;

			// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
			view->rgb_prev->Clear();

			const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN);
			trackingState->pose_d->SetFrom(&keyframe.pose);

			denseMapper->UpdateVisibleList(view, trackingState, volume, render_state, true);
			trackingController->Prepare(trackingState, volume, view, visualization_engine, render_state);
			trackingController->Track(trackingState, view);

			trackerResult = trackingState->trackerResult;
		}
	}

	bool didFusion = false;
	if ((trackerResult == CameraTrackingState::TRACKING_GOOD || !tracking_initialised) && (fusion_active) && (relocalization_count == 0)) {
		// fusion
		denseMapper->ProcessFrame(view, trackingState, volume, render_state);
		didFusion = true;
		if (processed_frame_count > 50) tracking_initialised = true;

		processed_frame_count++;
	}

	if (trackerResult == CameraTrackingState::TRACKING_GOOD || trackerResult == CameraTrackingState::TRACKING_POOR)
	{
		if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, volume, render_state);

		// raycast to renderState_canonical for tracking and free Rendering
		trackingController->Prepare(trackingState, volume, view, visualization_engine, render_state);

		if (addKeyframeIdx >= 0)
		{
			MemoryCopyDirection memoryCopyDirection =
					settings.device_type == MEMORYDEVICE_CUDA ? MemoryCopyDirection::CUDA_TO_CUDA : MemoryCopyDirection::CPU_TO_CPU;

			kfRaycast->SetFrom(*render_state->raycastImage, memoryCopyDirection);
		}
	}
	else *trackingState->pose_d = oldPose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
	const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().values[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().values[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif
    
    return trackerResult;
}

template <typename TVoxel, typename TIndex>
Vector2i BasicVoxelEngine<TVoxel,TIndex>::GetImageSize() const
{
	return render_state->raycastImage->dimensions;
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::GetImage(UChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, Intrinsics *intrinsics)
{
	if (view == nullptr) return;

	auto& settings = configuration::Get();

	out->Clear();

	switch (getImageType)
	{
	case BasicVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb.dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA)
			out->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
		break;
	case BasicVoxelEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth.dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA) view->depth.UpdateHostFromDevice();
		RenderingEngineBase<TVoxel, TIndex>::DepthToUchar4(out, view->depth);

		break;
	case BasicVoxelEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
	case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
		{
		// use current raycast or forward projection?
		IRenderingEngine::RenderRaycastSelection raycastType;
		if (trackingState->point_cloud_age <= 0) raycastType = IRenderingEngine::RENDER_FROM_OLD_RAYCAST;
		else raycastType = IRenderingEngine::RENDER_FROM_OLD_FORWARDPROJ;

		// what sort of image is it?
		IRenderingEngine::RenderImageType imageType;
		switch (getImageType) {
		case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
			imageType = IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE;
			break;
		case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
			imageType = IRenderingEngine::RENDER_COLOUR_FROM_NORMAL;
			break;
		case BasicVoxelEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
			imageType = IRenderingEngine::RENDER_COLOUR_FROM_VOLUME;
			break;
		default:
			imageType = IRenderingEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
		}

		visualization_engine->RenderImage(volume, trackingState->pose_d, &view->calibration_information.intrinsics_d, render_state, render_state->raycastImage, imageType, raycastType);

		ORUtils::Image<Vector4u>* source_image = nullptr;
		if (relocalization_count != 0) source_image = kfRaycast;
		else source_image = render_state->raycastImage;

		out->ChangeDims(source_image->dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA)
			out->SetFrom(*source_image, MemoryCopyDirection::CUDA_TO_CPU);
		else out->SetFrom(*source_image, MemoryCopyDirection::CPU_TO_CPU);

		break;
		}
	case BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	case BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
	{
		IRenderingEngine::RenderImageType type = IRenderingEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IRenderingEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IRenderingEngine::RENDER_COLOUR_FROM_NORMAL;
		else if (getImageType == BasicVoxelEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) type = IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE;

		if (render_state_freeview == nullptr)
		{
			render_state_freeview = new RenderState(out->dimensions, volume->GetParameters().near_clipping_distance, volume->GetParameters().far_clipping_distance, settings.device_type);
		}

		visualization_engine->FindVisibleBlocks(volume, pose, intrinsics, render_state_freeview);
		visualization_engine->CreateExpectedDepths(volume, pose, intrinsics, render_state_freeview);
		visualization_engine->RenderImage(volume, pose, intrinsics, render_state_freeview, render_state_freeview->raycastImage, type);

		if (settings.device_type == MEMORYDEVICE_CUDA)
			out->SetFrom(*render_state_freeview->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
		else out->SetFrom(*render_state_freeview->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
		break;
	}
	case FusionAlgorithm::InfiniTAM_IMAGE_UNKNOWN:
		break;
		case InfiniTAM_IMAGE_FREECAMERA_CANONICAL:break;
	};
}

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOnTracking() { tracking_active = true; }

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOffTracking() { tracking_active = false; }

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOnIntegration() { fusion_active = true; }

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOffIntegration() { fusion_active = false; }

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOnMainProcessing() { main_processing_active = true; }

template <typename TVoxel, typename TIndex>
void BasicVoxelEngine<TVoxel,TIndex>::TurnOffMainProcessing() { main_processing_active = false; }
