// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "MultiEngine.h"

#include "../ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ViewBuilder/ViewBuilderFactory.h"
#include "../Rendering/RenderingEngineFactory.h"
#include "../Rendering/MultiVisualizationEngineFactory.h"
#include "../../CameraTrackers/CameraTrackerFactory.h"

#include "../../../MiniSlamGraphLib/QuaternionHelpers.h"
#include "../../Objects/RenderStates/RenderStateMultiScene.h"

using namespace ITMLib;

//#define DEBUG_MULTISCENE

// number of nearest neighbours to find in the loop closure detection
static const int k_loopcloseneighbours = 1;

// maximum distance reported by LCD library to attempt relocalisation
static const float F_maxdistattemptreloc = 0.05f;

// loop closure global adjustment runs on a separate thread
static const bool separateThreadGlobalAdjustment = true;


//TODO Clean up & spruce up this MultiEngine shitty code if it has any utility for anyone out there.

template <typename TVoxel, typename TIndex>
MultiEngine<TVoxel, TIndex>::MultiEngine(const RGBD_CalibrationInformation& calib, Vector2i imgSize_rgb, Vector2i imgSize_d){
	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	auto& settings = configuration::Get();

	const MemoryDeviceType deviceType = settings.device_type;
	lowLevelEngine = ImageProcessingEngineFactory::Build(deviceType);
	viewBuilder = ViewBuilderFactory::Build(calib, deviceType);
	visualization_engine = RenderingEngineFactory::Build<TVoxel, TIndex>(deviceType);

	tracker = CameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, lowLevelEngine, imuCalibrator,
	                                                settings.general_voxel_volume_parameters);
	trackingController = new CameraTrackingController(tracker);
	trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	mapManager = new VoxelMapGraphManager<TVoxel, TIndex>(visualization_engine, denseMapper, trackedImageSize);
	mActiveDataManager = new ActiveMapManager(mapManager);
	mActiveDataManager->initiateNewLocalMap(true);
	denseMapper = new DenseMapper<TVoxel, TIndex>();

	meshingEngine = NULL;
	if (settings.create_meshing_engine)
		meshingEngine = MultiMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType, mapManager->getLocalMap(0)->volume->index);

	renderState_freeview = NULL; //will be created by the Rendering engine
	imuCalibrator = new ITMIMUCalibrator_iPad();
	freeviewLocalMapIdx = 0;

	//TODO	tracker->UpdateInitialPose(allData[0]->trackingState);

	view = NULL; // will be allocated by the view builder

	relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings.general_voxel_volume_parameters.near_clipping_distance, settings.general_voxel_volume_parameters.far_clipping_distance), 0.1f, 1000, 4);

	mGlobalAdjustmentEngine = new GlobalAdjustmentEngine();
	mScheduleGlobalAdjustment = false;
	if (separateThreadGlobalAdjustment) mGlobalAdjustmentEngine->startSeparateThread();

	multiVisualizationEngine = MultiVisualizationEngineFactory::MakeVisualizationEngine<TVoxel,TIndex>(deviceType);
	renderState_multiscene = NULL;
}

template <typename TVoxel, typename TIndex>
MultiEngine<TVoxel, TIndex>::~MultiEngine()
{
	if (renderState_multiscene != NULL) delete renderState_multiscene;

	delete mGlobalAdjustmentEngine;
	delete mActiveDataManager;
	delete mapManager;

	if (renderState_freeview != NULL) delete renderState_freeview;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	if (view != NULL) delete view;

	delete visualization_engine;

	delete relocaliser;

	delete multiVisualizationEngine;
}

template <typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::changeFreeviewLocalMapIdx(ORUtils::SE3Pose *pose, int newIdx)
{
	//if ((newIdx < 0) || ((unsigned)newIdx >= mapManager->numLocalMaps())) return;

	if (newIdx < -1) newIdx = (int)mapManager->numLocalMaps() - 1;
	if ((unsigned)newIdx >= mapManager->numLocalMaps()) newIdx = -1;

	ORUtils::SE3Pose trafo = mapManager->findTransformation(freeviewLocalMapIdx, newIdx);
	pose->SetM(pose->GetM() * trafo.GetInvM());
	pose->Coerce();
	freeviewLocalMapIdx = newIdx;
}

template <typename TVoxel, typename TIndex>
CameraTrackingState* MultiEngine<TVoxel, TIndex>::GetTrackingState()
{
	int idx = mActiveDataManager->findPrimaryLocalMapIdx();
	if (idx < 0) idx = 0;
	return mapManager->getLocalMap(idx)->trackingState;
}

// -whenever a new local scene is added, add to list of "to be established 3D relations"
// - whenever a relocalisation is detected, add to the same list, preserving any existing information on that 3D relation
//
// - for all 3D relations to be established :
// -attempt tracking in both scenes
// - if success, add to list of new candidates
// - if less than n_overlap "new candidates" in more than n_reloctrialframes frames, discard
// - if at least n_overlap "new candidates" :
// 	- try to compute_allocated 3D relation, weighting old information accordingly
//	- if outlier ratio below p_relation_outliers and at least n_overlap inliers, success

struct TodoListEntry {
	TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare)
		: dataId(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), preprepare(false) {}
	TodoListEntry() {}
	int dataId;
	bool track;
	bool fusion;
	bool prepare;
	bool preprepare;
};

template <typename TVoxel, typename TIndex>
CameraTrackingState::TrackingResult MultiEngine<TVoxel, TIndex>::ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, IMUMeasurement *imuMeasurement)
{
	auto& settings = configuration::Get();
	std::vector<TodoListEntry> todoList;
	CameraTrackingState::TrackingResult primaryLocalMapTrackingResult = CameraTrackingState::TrackingResult::TRACKING_FAILED;

	// prepare image and turn it into a depth image
	if (imuMeasurement == NULL)
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, false, true);
	else
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings.use_threshold_filter,
		                        settings.use_bilateral_filter, imuMeasurement,
		                        false, true);

	// find primary data, if available
	int primaryDataIdx = mActiveDataManager->findPrimaryDataIdx();

	// if there is a "primary data index", process it
	if (primaryDataIdx >= 0) todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true));

	// after primary local map, make sure to process all relocalizations, new scenes and loop closures
	for (int i = 0; i < mActiveDataManager->numActiveLocalMaps(); ++i)
	{
		switch (mActiveDataManager->getLocalMapType(i))
		{
		case ActiveMapManager::NEW_LOCAL_MAP: todoList.push_back(TodoListEntry(i, true, true, true));
		case ActiveMapManager::LOOP_CLOSURE: todoList.push_back(TodoListEntry(i, true, false, true));
		case ActiveMapManager::RELOCALISATION: todoList.push_back(TodoListEntry(i, true, false, true));
		default: break;
		}
	}

	// finally, once all is done, call the loop closure detection engine
	todoList.push_back(TodoListEntry(-1, false, false, false));

	bool primaryTrackingSuccess = false;
	for (size_t i = 0; i < todoList.size(); ++i)
	{
		// - first pass of the todo list is for primary local map and ongoing relocalization and loop closure attempts
		// - an element with id -1 marks the end of the first pass, a request to call the loop closure detection engine, and
		//   the start of the second pass
		// - second tracking pass will be about newly detected loop closures, relocalizations, etc.

		if (todoList[i].dataId == -1)
		{
#ifdef DEBUG_MULTISCENE
			fprintf(stderr, " Reloc(%i)", primaryTrackingSuccess);
#endif
			int NN[k_loopcloseneighbours]; float distances[k_loopcloseneighbours];
			view->depth.UpdateHostFromDevice();

			//primary map index
			int primaryLocalMapIdx = -1;
			if (primaryDataIdx >= 0) primaryLocalMapIdx = mActiveDataManager->getLocalMapIndex(primaryDataIdx);

			//check if relocaliser has fired
			ORUtils::SE3Pose *pose = primaryLocalMapIdx >= 0 ? mapManager->getLocalMap(primaryLocalMapIdx)->trackingState->pose_d : NULL;
			bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, pose, primaryLocalMapIdx, k_loopcloseneighbours, NN, distances, primaryTrackingSuccess);

			//frame not added and tracking failed -> we need to relocalise
			if (!hasAddedKeyframe)
			{
				for (int j = 0; j < k_loopcloseneighbours; ++j)
				{
					if (distances[j] > F_maxdistattemptreloc) continue;
					const FernRelocLib::PoseDatabase::PoseInScene & keyframe = relocaliser->RetrievePose(NN[j]);
					int newDataIdx = mActiveDataManager->initiateNewLink(keyframe.sceneIdx, keyframe.pose, (primaryLocalMapIdx < 0));
					if (newDataIdx >= 0)
					{
						TodoListEntry todoItem(newDataIdx, true, false, true);
						todoItem.preprepare = true;
						todoList.push_back(todoItem);
					}
				}
			}

			continue;
		}

		LocalMap<TVoxel, TIndex> *currentLocalMap = NULL;
		int currentLocalMapIdx = mActiveDataManager->getLocalMapIndex(todoList[i].dataId);
		currentLocalMap = mapManager->getLocalMap(currentLocalMapIdx);

		// if a new relocalization/loop closure is started, this will do the initial raycasting before tracking can start
		if (todoList[i].preprepare)
		{
			denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->volume, currentLocalMap->renderState);
			trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->volume, view, visualization_engine, currentLocalMap->renderState);
		}

		if (todoList[i].track)
		{
			int dataId = todoList[i].dataId;

#ifdef DEBUG_MULTISCENE
			int blocksInUse = currentLocalMap->volume->index.Getsource_volume() - currentLocalMap->volume->index.GetLastFreeBlockListId() - 1;
			fprintf(stderr, " %i%s (%i)", currentLocalMapIdx, (todoList[i].dataId == primaryDataIdx) ? "*" : "", blocksInUse);
#endif

			// actual tracking
			ORUtils::SE3Pose oldPose(*(currentLocalMap->trackingState->pose_d));
			trackingController->Track(currentLocalMap->trackingState, view);

			// tracking is allowed to be poor only in the primary scenes. 
			CameraTrackingState::TrackingResult trackingResult = currentLocalMap->trackingState->trackerResult;
			if (mActiveDataManager->getLocalMapType(dataId) != ActiveMapManager::PRIMARY_LOCAL_MAP)
				if (trackingResult == CameraTrackingState::TRACKING_POOR) trackingResult = CameraTrackingState::TRACKING_FAILED;

			// actions on tracking result for all scenes TODO: incorporate behaviour on tracking failure from settings
			if (trackingResult != CameraTrackingState::TRACKING_GOOD) todoList[i].fusion = false;

			if (trackingResult == CameraTrackingState::TRACKING_FAILED)
			{
				todoList[i].prepare = false;
				*(currentLocalMap->trackingState->pose_d) = oldPose;
			}

			// actions on tracking result for primary local map
			if (mActiveDataManager->getLocalMapType(dataId) == ActiveMapManager::PRIMARY_LOCAL_MAP)
			{
				primaryLocalMapTrackingResult = trackingResult;

				if (trackingResult == CameraTrackingState::TRACKING_GOOD) primaryTrackingSuccess = true;

				// we need to relocalise in the primary local map
				else if (trackingResult == CameraTrackingState::TRACKING_FAILED)
				{
					primaryDataIdx = -1;
					todoList.resize(i + 1);
					todoList.push_back(TodoListEntry(-1, false, false, false));
				}
			}

			mActiveDataManager->recordTrackingResult(dataId, trackingResult, primaryTrackingSuccess);
		}

		// fusion in any subscene as long as tracking is good for the respective subscene
		if (todoList[i].fusion) denseMapper->ProcessFrame(view, currentLocalMap->trackingState, currentLocalMap->volume, currentLocalMap->renderState);
		else if (todoList[i].prepare) denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->volume, currentLocalMap->renderState);

		// raycast to renderState_canonical for tracking and free Rendering
		if (todoList[i].prepare) trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->volume, view, visualization_engine, currentLocalMap->renderState);
	}

	mScheduleGlobalAdjustment |= mActiveDataManager->maintainActiveData();

	if (mScheduleGlobalAdjustment)
	{
		if (mGlobalAdjustmentEngine->updateMeasurements(*mapManager))
		{
			if (separateThreadGlobalAdjustment) mGlobalAdjustmentEngine->wakeupSeparateThread();
			else mGlobalAdjustmentEngine->runGlobalAdjustment();

			mScheduleGlobalAdjustment = false;
		}
	}
	mGlobalAdjustmentEngine->retrieveNewEstimates(*mapManager);

	return primaryLocalMapTrackingResult;
}

template <typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::SaveVolumeToMesh(const std::string& path)
{
	if (meshingEngine == NULL) return;
	Mesh mesh = meshingEngine->MeshVolume(*mapManager);
	mesh.WritePLY(path);
}

template <typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::SaveToFile()
{

}

template <typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::LoadFromFile()
{

}

template <typename TVoxel, typename TIndex>
Vector2i MultiEngine<TVoxel, TIndex>::GetImageSize() const
{
	return trackedImageSize;
}

template <typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::GetImage(UChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose, Intrinsics *intrinsics)
{
	if (view == NULL) return;
	auto& settings = configuration::Get();

	out->Clear();

	switch (getImageType)
	{
	case MultiEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb.dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA)
			out->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
		break;
	case MultiEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth.dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA) view->depth.UpdateHostFromDevice();
		RenderingEngineBase<TVoxel, TIndex>::DepthToUchar4(out, view->depth);
		break;
    case MultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME: //TODO: add colour rendering
	case MultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	case MultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
	case MultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
	{
		int VisualizationLocalMapIdx = mActiveDataManager->findBestVisualizationLocalMapIdx();
		if (VisualizationLocalMapIdx < 0) break; // TODO: clear image? what else to do when tracking is lost?

		LocalMap<TVoxel, TIndex> *activeLocalMap = mapManager->getLocalMap(VisualizationLocalMapIdx);

		IRenderingEngine::RenderRaycastSelection raycastType;
		if (activeLocalMap->trackingState->point_cloud_age <= 0) raycastType = IRenderingEngine::RENDER_FROM_OLD_RAYCAST;
		else raycastType = IRenderingEngine::RENDER_FROM_OLD_FORWARDPROJ;

		IRenderingEngine::RenderImageType imageType;
		switch (getImageType)
		{
		case MultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
			imageType = IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE;
			break;
		case MultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
			imageType = IRenderingEngine::RENDER_COLOUR_FROM_NORMAL;
			break;
		default:
			imageType = IRenderingEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
		}

		visualization_engine->RenderImage(activeLocalMap->volume, activeLocalMap->trackingState->pose_d, &view->calibration_information.intrinsics_d, activeLocalMap->renderState, activeLocalMap->renderState->raycastImage, imageType, raycastType);

		ORUtils::Image<Vector4u> *srcImage = activeLocalMap->renderState->raycastImage;
		out->ChangeDims(srcImage->dimensions);
		if (settings.device_type == MEMORYDEVICE_CUDA)
			out->SetFrom(*srcImage, MemoryCopyDirection::CUDA_TO_CPU);
		else out->SetFrom(*srcImage, MemoryCopyDirection::CPU_TO_CPU);
		break;
	}
	case MultiEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	case MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE:
	{
		IRenderingEngine::RenderImageType type = IRenderingEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IRenderingEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IRenderingEngine::RENDER_COLOUR_FROM_NORMAL;
		else if (getImageType == MultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) type = IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE;

		if (freeviewLocalMapIdx >= 0){
			LocalMap<TVoxel, TIndex> *activeData = mapManager->getLocalMap(freeviewLocalMapIdx);
			if (renderState_freeview == NULL) renderState_freeview =
					new RenderStateMultiScene<TVoxel,TIndex>(out->dimensions, activeData->volume->GetParameters().near_clipping_distance,
					                                         activeData->volume->GetParameters().far_clipping_distance, settings.device_type);

			visualization_engine->FindVisibleBlocks(activeData->volume, pose, intrinsics, renderState_freeview);
			visualization_engine->CreateExpectedDepths(activeData->volume, pose, intrinsics, renderState_freeview);
			visualization_engine->RenderImage(activeData->volume, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*renderState_freeview->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*renderState_freeview->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
		}
		else
		{
			const VoxelVolumeParameters& params = mapManager->getLocalMap(0)->volume->GetParameters();
			if (renderState_multiscene == nullptr) renderState_multiscene =
						new RenderStateMultiScene<TVoxel, TIndex>(out->dimensions, params.near_clipping_distance,
						                                          params.far_clipping_distance, settings.device_type);
			multiVisualizationEngine->PrepareRenderState(*mapManager, renderState_multiscene);
			multiVisualizationEngine->CreateExpectedDepths(*mapManager, pose, intrinsics, renderState_multiscene);
			multiVisualizationEngine->RenderImage(pose, intrinsics, renderState_multiscene, renderState_multiscene->raycastImage, type);
			if (settings.device_type == MEMORYDEVICE_CUDA)
				out->SetFrom(*renderState_multiscene->raycastImage, MemoryCopyDirection::CUDA_TO_CPU);
			else out->SetFrom(*renderState_multiscene->raycastImage, MemoryCopyDirection::CPU_TO_CPU);
		}

		break;
	}
	case MultiEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOnTracking() {
	std::cerr << "Tracking on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOffTracking() {
	std::cerr << "Tracking on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOnIntegration() {
	std::cerr << "Fusion on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOffIntegration() {
	std::cerr << "Fusion on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOnMainProcessing() {
	std::cerr << "Main processing on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::TurnOffMainProcessing() {
	std::cerr << "Main processing on/off switch not available in " __FILE__ "." << std::endl;
}

template<typename TVoxel, typename TIndex>
void MultiEngine<TVoxel, TIndex>::ResetAll() {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}
