// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Mappers/DenseSurfelMapper.h"
#include "FusionAlgorithm.h"
#include "CameraTrackingController.h"
#include "../ImageProcessing/Interface/ImageProcessingEngineInterface.h"
#include "../ViewBuilder/Interface/ViewBuilder.h"
#include "../Rendering/Interface/SurfelVisualizationEngine.h"
#include "../../Objects/Misc/IMUCalibrator.h"

#include "../../../FernRelocLib/Relocaliser.h"

namespace ITMLib{
	template <typename TSurfel>
	class BasicSurfelEngine : public FusionAlgorithm{
	private: // instance variables
		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		ImageProcessingEngineInterface *lowLevelEngine;
		SurfelVisualizationEngine<TSurfel> *surfelVisualizationEngine;

		ViewBuilder *viewBuilder;
		DenseSurfelMapper<TSurfel> *denseSurfelMapper;
		CameraTrackingController *trackingController;

		SurfelScene<TSurfel> *surfelScene;
		SurfelRenderState *surfelRenderState_live;
		SurfelRenderState *surfelRenderState_freeview;

		CameraTracker *tracker;
		IMUCalibrator *imuCalibrator;

		FernRelocLib::Relocaliser<float> *relocaliser;
		UChar4Image *kfRaycast;

		/// Pointer for storing the current input frame
		View *view;

		/// Pointer to the current camera pose and additional tracking information
		CameraTrackingState *trackingState;

		static typename SurfelVisualizationEngine<TSurfel>::RenderImageType ToSurfelImageType(GetImageType getImageType);

	public:
		View* GetView() { return view; }
		CameraTrackingState* GetTrackingState() { return trackingState; }

		CameraTrackingState::TrackingResult ProcessFrame(UChar4Image *rgbImage, ShortImage *rawDepthImage, IMUMeasurement *imuMeasurement = nullptr);

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveVolumeToMesh(const std::string& path);

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile();
		void LoadFromFile();

		/// Get a result image as output
		Vector2i GetImageSize() const;

		void GetImage(UChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = nullptr, Intrinsics *intrinsics = nullptr);

		/// switch for turning tracking on/off
		void TurnOnTracking() override;
		void TurnOffTracking() override;

		/// switch for turning integration on/off
		void TurnOnIntegration() override;
		void TurnOffIntegration() override;

		/// switch for turning main processing on/off
		void TurnOnMainProcessing() override;
		void TurnOffMainProcessing() override;

		virtual bool GetMainProcessingOn() const override;

		/// resets the scene and the tracker
		void ResetAll();

		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		BasicSurfelEngine(const RGBD_CalibrationInformation& calib, Vector2i imgSize_rgb, Vector2i imgSize_d);
		~BasicSurfelEngine();
	};
}
