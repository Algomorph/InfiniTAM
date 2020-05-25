// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Mappers/DenseMapper.h"
#include "MainEngine.h"
#include "CameraTrackingController.h"
#include "../LowLevel/Interface/LowLevelEngine.h"
#include "../Meshing/Interface/MeshingEngine.h"
#include "../ViewBuilding/Interface/ViewBuilder.h"
#include "../Rendering/Interface/RenderingEngine.h"
#include "../../Objects/Misc/IMUCalibrator.h"

#include "../../../FernRelocLib/Relocaliser.h"

namespace ITMLib
{
	template <typename TVoxel, typename TIndex>
	class BasicVoxelEngine : public MainEngine
	{
	private:
		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		LowLevelEngine *lowLevelEngine;
		RenderingEngineBase<TVoxel, TIndex> *visualizationEngine;

		MeshingEngine<TVoxel, TIndex> *meshingEngine;

		ViewBuilder *viewBuilder;
		DenseMapper<TVoxel, TIndex> *denseMapper;
		CameraTrackingController *trackingController;

		VoxelVolume<TVoxel, TIndex> *volume;
		RenderState *render_state;
		RenderState *renderState_freeview;

		CameraTracker *tracker;
		IMUCalibrator *imuCalibrator;

		FernRelocLib::Relocaliser<float> *relocaliser;
		UChar4Image *kfRaycast;

		/// Pointer for storing the current input frame
		View *view;

		/// Pointer to the current camera pose and additional tracking information
		CameraTrackingState *trackingState;

	public:
		View* GetView() { return view; }
		CameraTrackingState* GetTrackingState() { return trackingState; }

		/// Gives access to the internal world representation
		VoxelVolume<TVoxel, TIndex>* GetScene() { return volume; }

		CameraTrackingState::TrackingResult ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, IMUMeasurement *imuMeasurement = NULL);

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName);

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile();
		void LoadFromFile();

		/// Get a result image as output
		Vector2i GetImageSize() const;

		void GetImage(UChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, Intrinsics *intrinsics = NULL);

		/// switch for turning tracking on/off
		void TurnOnTracking() override;
		void TurnOffTracking() override;

		/// switch for turning integration on/off
		void TurnOnIntegration() override;
		void TurnOffIntegration() override;

		/// switch for turning main processing on/off
		void TurnOnMainProcessing() override;
		void TurnOffMainProcessing() override;

		/// resets the scene and the tracker
		void ResetAll();

		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		BasicVoxelEngine(const RGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d);
		~BasicVoxelEngine();
	};
}
