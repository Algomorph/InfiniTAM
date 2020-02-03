// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// Modified code: Copyright 2020 Gregory Kramida

#pragma once

//local
#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Engines/Main/MainEngine.h"
#include "../../ITMLib/Objects/Tracking/CameraTrackingState.h"
#include "../../ITMLib/Utils/Configuration.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"
#include "../../ITMLib/Utils/FileIO/DynamicFusionLogger.h"

//stdlib
#include <vector>

//boost
#include <boost/filesystem.hpp>


namespace InfiniTAM {
namespace Engine { //TODO: retching from the overuse of the word "Engine"
class UIEngine_BPO {
private:

	enum MainLoopAction {
		PROCESS_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, EXIT, PROCESS_N_FRAMES
	} mainLoopAction;

	struct UIColourMode {
		const char* name;
		ITMLib::MainEngine::GetImageType type;

		UIColourMode(const char* _name, ITMLib::MainEngine::GetImageType _type)
				: name(_name), type(_type) {}
	};
	std::vector<UIColourMode> colourModes_main, colourModes_freeview;

	int currentColourMode;

	int autoIntervalFrameStart;
	int number_of_frames_to_process_after_launch;
	bool exit_after_automatic_run = false;
	bool save_after_automatic_run = false;
	int startedProcessingFromFrameIx = 0;

	InputSource::ImageSourceEngine* imageSource;
	InputSource::IMUSourceEngine* imuSource;
	ITMLib::MainEngine* mainEngine;

	StopWatchInterface* timer_instant;
	StopWatchInterface* timer_average;

	// For UI layout
	static const int NUM_WIN = 3;
	Vector4f winReg[NUM_WIN]; // (x1, y1, x2, y2)
	Vector2i winSize;
	uint textureId[NUM_WIN];
	ITMUChar4Image* outImage[NUM_WIN];
	ITMLib::MainEngine::GetImageType outImageType[NUM_WIN];

	ITMUChar4Image* inputRGBImage;
	ITMShortImage* inputRawDepthImage;
	ITMLib::IMUMeasurement* inputIMUMeasurement;

	bool freeviewActive;
	bool integrationActive;
	ORUtils::SE3Pose freeviewPose;
	ITMLib::Intrinsics freeviewIntrinsics;

	int mouseState;
	Vector2i mouseLastClick;
	bool mouseWarped; // To avoid the extra motion generated by glutWarpPointer

	int currentFrameNo;
	bool isRecordingImages;

	InputSource::FFMPEGWriter* reconstructionVideoWriter = nullptr;
	InputSource::FFMPEGWriter* rgbVideoWriter = nullptr;
	InputSource::FFMPEGWriter* depthVideoWriter = nullptr;
public:
	static UIEngine_BPO& Instance() {
		static UIEngine_BPO instance;
		return instance;
	}

	static void GlutDisplayFunction();
	static void GlutIdleFunction();
	static void GlutKeyUpFunction(unsigned char key, int x, int y);
	static void GlutMouseButtonFunction(int button, int state, int x, int y);
	static void GlutMouseMoveFunction(int x, int y);
	static void GlutMouseWheelFunction(int button, int dir, int x, int y);

	const Vector2i& GetWindowSize() const { return winSize; }

	float processedTime;
	int processedFrameNo;
	ITMLib::CameraTrackingState::TrackingResult trackingResult;
	std::string output_path;
	bool needsRefresh;

	bool allocateGPU;
	bool shutdownRequested = false;
	ITMUChar4Image* saveImage;
	ITMLib::DynamicFusionLogger_Interface* logger;
	ITMLib::configuration::IndexingMethod indexingMethod;

	void Initialize(int& argc, char** argv, InputSource::ImageSourceEngine* imageSource, InputSource::IMUSourceEngine* imuSource,
	                ITMLib::MainEngine* mainEngine, const ITMLib::configuration::Configuration& configuration,
	                ITMLib::DynamicFusionLogger_Interface* logger);
	void Shutdown();

	void Run();
	void PrintProcessingFrameHeader() const;
	void ProcessFrame();

	void GetScreenshot(ITMUChar4Image* dest) const;
	void SaveScreenshot(const char* filename) const;

	void SkipFrames(int numberOfFramesToSkip);
	void RecordCurrentReconstructionFrameToVideo();
	void RecordDepthAndRGBInputToVideo();
	void RecordDepthAndRGBInputToImages();
	int GetCurrentFrameIndex() const;
	std::string GenerateNextFrameOutputPath() const;
	std::string GenerateCurrentFrameOutputDirectory() const;
};
} // namespace Engine -- bleh?
} // namespace InfiniTAM
