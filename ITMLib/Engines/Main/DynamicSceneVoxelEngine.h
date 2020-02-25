// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified/expanded work copyright 2017-2019 Gregory Kramida

#pragma once

#include <array>
#include "Mappers/DenseDynamicMapper.h"
#include "MainEngine.h"
#include "CameraTrackingController.h"
#include "../LowLevel/Interface/LowLevelEngine.h"
#include "../Meshing/Interface/MeshingEngine.h"
#include "../ViewBuilding/Interface/ViewBuilder.h"
#include "../Visualization/Interface/VisualizationEngine.h"
#include "../../Objects/Misc/IMUCalibrator.h"

#include "../../../FernRelocLib/Relocaliser.h"
#include "../../CameraTrackers/Interface/CameraTracker.h"


namespace ITMLib
{
	template <typename TVoxel, typename TWarp, typename TIndex>
	class DynamicSceneVoxelEngine : public MainEngine
	{
	public:


		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		DynamicSceneVoxelEngine(const RGBDCalib& calibration_info, Vector2i rgb_image_size, Vector2i depth_image_size);
		~DynamicSceneVoxelEngine() override;

		View* GetView() override { return view; }
		CameraTrackingState* GetTrackingState() override { return tracking_state; }

		CameraTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, IMUMeasurement *imuMeasurement = nullptr) override;

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName) override ;

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile() override;
		void SaveToFile(const std::string& path) override;
		void LoadFromFile() override;
		void LoadFromFile(const std::string& path) override;

		/// Get a result image as output
		Vector2i GetImageSize() const override;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType,
		              ORUtils::SE3Pose *pose = nullptr, Intrinsics *intrinsics = nullptr) override;

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
		static const int live_scene_count = 2;
		//TODO need better function separation here, "begin" is logically too arbitrary and does too many things
		void HandlePotentialCameraTrackingFailure();

		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		configuration::Configuration settings;

		LowLevelEngine* low_level_engine;
		VisualizationEngine<TVoxel, TIndex>* visualization_engine;

		MeshingEngine<TVoxel, TIndex>* meshing_engine;

		ViewBuilder* view_builder;
		DenseDynamicMapper<TVoxel, TWarp, TIndex>* denseMapper;
		CameraTrackingController* camera_tracking_controller;

		VoxelVolume<TVoxel, TIndex>* canonical_volume;
		VoxelVolume<TVoxel, TIndex>** live_volumes;
		VoxelVolume<TWarp, TIndex>* warp_field;
		RenderState* canonical_render_state;
		RenderState* live_render_state;
		RenderState* freeview_render_state;

		CameraTracker* tracker;
		IMUCalibrator* imuCalibrator;

		FernRelocLib::Relocaliser<float>* relocaliser;
		ITMUChar4Image* kfRaycast;

		/// Pointer for storing the current input frame
		View* view;

		/// Pointer to the current camera pose and additional tracking information
		CameraTrackingState* tracking_state;
		CameraTrackingState::TrackingResult last_tracking_result;
		bool fusion_succeeded;
		ORUtils::SE3Pose previousFramePose;
	};
}
