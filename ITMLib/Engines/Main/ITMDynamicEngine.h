// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified/expanded work copyright 2017-2019 Gregory Kramida

#pragma once

#include <array>
#include "DenseDynamicMapper.h"
#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Meshing/Interface/ITMMeshingEngine.h"
#include "../ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Visualization/Interface/ITMVisualisationEngine.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"

#include "../../../FernRelocLib/Relocaliser.h"
#include "../../CameraTrackers/Interface/ITMCameraTracker.h"


namespace ITMLib
{
	template <typename TVoxel, typename TWarp, typename TIndex>
	class ITMDynamicEngine : public ITMMainEngine
	{
	public:


		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		ITMDynamicEngine(const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d);
		~ITMDynamicEngine() override;

		ITMView* GetView() override { return view; }
		ITMTrackingState* GetTrackingState() override { return trackingState; }

		ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = nullptr) override;

		//_DEBUG for visual debugging
		void BeginProcessingFrameInStepByStepMode(ITMUChar4Image* rgbImage, ITMShortImage* rawDepthImage,
		                                          ITMIMUMeasurement* imuMeasurement = nullptr);
		bool UpdateCurrentFrameSingleStep();
		ITMTrackingState::TrackingResult GetStepByStepTrackingResult();

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName) override ;

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile() override;
		void LoadFromFile() override;

		/// Get a result image as output
		Vector2i GetImageSize() const override;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType,
		              ORUtils::SE3Pose *pose = nullptr, ITMIntrinsics *intrinsics = nullptr) override;

		/// resets the scene and the tracker
		void resetAll() override;

		/// switch for turning tracking on/off
		void turnOnTracking() override;
		void turnOffTracking() override;

		/// switch for turning integration on/off
		void turnOnIntegration() override;
		void turnOffIntegration() override;

		/// switch for turning main processing on/off
		void turnOnMainProcessing() override;
		void turnOffMainProcessing() override;

	private:
		void Reset();
		void InitializeScenes();
		static const int liveSceneCount = 2;
		void BeginProcessingFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = nullptr);

		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		ITMLowLevelEngine* lowLevelEngine;
		ITMVisualisationEngine<TVoxel, TIndex>* liveVisualisationEngine;
		ITMVisualisationEngine<TVoxel, TIndex>* canonicalVisualisationEngine;

		ITMMeshingEngine<TVoxel, TIndex>* meshingEngine;

		ITMViewBuilder* viewBuilder;
		DenseDynamicMapper<TVoxel, TWarp, TIndex>* denseMapper;
		ITMTrackingController* cameraTrackingController;

		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene;
		ITMVoxelVolume<TVoxel, TIndex>** liveScenes;
		ITMVoxelVolume<TWarp, TIndex>* warpField;
		ITMRenderState* renderState_live;
		ITMRenderState* renderState_freeview;

		ITMCameraTracker* tracker;
		ITMIMUCalibrator* imuCalibrator;

		FernRelocLib::Relocaliser<float>* relocaliser;
		ITMUChar4Image* kfRaycast;

		/// Pointer for storing the current input frame
		ITMView* view;

		/// Pointer to the current camera pose and additional tracking information
		ITMTrackingState* trackingState;
		ITMTrackingState::TrackingResult lastTrackerResult;
		bool fusionSucceeded;
		bool canFuse;
		ORUtils::SE3Pose previousFramePose;
	};
}
