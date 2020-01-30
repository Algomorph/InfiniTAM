// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "UIEngine_BPO.h"

#include <string.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else

#include <GL/glut.h>

#endif

#ifdef FREEGLUT

#include <GL/freeglut.h>

#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#ifdef WITH_VTK
//VTK
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>
#endif

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Engines/Main/ITMBasicEngine.h"
#include "../../ITMLib/Engines/Main/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Engines/Main/ITMMultiEngine.h"
#include "../../ITMLib/Engines/Main/ITMDynamicEngine.h"

#include "../../ORUtils/FileUtils.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Utils/Analytics/ITMBenchmarkUtils.h"
#include "../../ITMLib/Utils/ITMPrintHelpers.h"

#ifdef WITH_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif


//TODO: we should never have to downcast the main engine to some other engine type, architecture needs to be altered
// (potentially by introducting empty method stubs) -Greg (GitHub:Algomorph)

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


namespace bench = ITMLib::Bench;


/**
 * \brief Initialize the UIEngine using the specified settings
 * \param argc arguments to the main function
 * \param argv number of arguments to the main function
 * \param imageSource source for images
 * \param imuSource source for IMU data
 * \param mainEngine main engine to process the frames
 * \param outFolder output folder for saving results
 * \param deviceType type of device to use for some tasks
 * \param number_of_frames_to_process_after_launch automatically process this number of frames after launching the UIEngine,
 * \param index_of_first_frame skip this number of frames before beginning to process
 * \param recordReconstructionResult start recording the reconstruction result into a video files as soon as the next frame is processed
 * set interval to this number of frames
 */
void UIEngine_BPO::Initialize(int& argc, char** argv,
							  InputSource::ImageSourceEngine* imageSource,
                              InputSource::IMUSourceEngine* imuSource,
                              ITMLib::ITMMainEngine* mainEngine,

                              const configuration::Configuration& configuration,
                              ITMLib::ITMDynamicFusionLogger_Interface* logger) {

	//TODO: somehow incorporate the following "constant" settings into Configuration struct, Configuration.h in ITMLib
	const bool fix_camera = false;
	const bool load_volume_before_automatic_run = false;
	const bool save_volume_after_automatic_run = false;


	this->logger = logger;
	this->indexingMethod = configuration.indexing_method;

	this->save_after_automatic_run = save_volume_after_automatic_run;
	this->exit_after_automatic_run = configuration.automatic_run_settings.exit_after_automatic_processing;

	this->freeviewActive = true;
	this->integrationActive = true;
	this->currentColourMode = 0;
	this->number_of_frames_to_process_after_launch = configuration.automatic_run_settings.number_of_frames_to_process;

	this->colourModes_main.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
	this->colourModes_main.emplace_back("integrated colours", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
	this->colourModes_main.emplace_back("surface normals", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL);
	this->colourModes_main.emplace_back("confidence", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE);

	this->colourModes_freeview.emplace_back("canonical", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL);
	this->colourModes_freeview.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED);
	this->colourModes_freeview.emplace_back("integrated colours",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME);
	this->colourModes_freeview.emplace_back("surface normals",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL);
	this->colourModes_freeview.emplace_back("confidence",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE);

	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = mainEngine;
	this->output_path = configuration.paths.output_path;

	int textHeight = 60; // Height of text area, 2 lines

	winSize.x = (int) (1.5f * (float) (imageSource->getDepthImageSize().x));
	winSize.y = imageSource->getDepthImageSize().y + textHeight;
	float h1 = textHeight / (float) winSize.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->isRecordingImages = false;
	this->currentFrameNo = 0;
	this->rgbVideoWriter = nullptr;
	this->depthVideoWriter = nullptr;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(winSize.x, winSize.y);
	glutCreateWindow("InfiniTAM");
	glGenTextures(NUM_WIN, textureId);

	glutDisplayFunc(UIEngine_BPO::GlutDisplayFunction);
	glutKeyboardUpFunc(UIEngine_BPO::GlutKeyUpFunction);
	glutMouseFunc(UIEngine_BPO::GlutMouseButtonFunction);
	glutMotionFunc(UIEngine_BPO::GlutMouseMoveFunction);
	glutIdleFunc(UIEngine_BPO::GlutIdleFunction);

#ifdef FREEGLUT
	glutMouseWheelFunc(UIEngine_BPO::GlutMouseWheelFunction);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	allocateGPU = configuration.device_type == MEMORYDEVICE_CUDA;

	for (int w = 0; w < NUM_WIN; w++) {
		outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), true, false);


	outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->noDims == Vector2i(0, 0)) outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN;


	autoIntervalFrameStart = 0;
	mouseState = 0;
	mouseWarped = false;
	needsRefresh = false;
	processedFrameNo = 0;
	processedTime = 0.0f;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);
	if (configuration.automatic_run_settings.index_of_frame_to_start_at > 0) {
		printf("Skipping the first %d frames.\n", configuration.automatic_run_settings.index_of_frame_to_start_at);
		SkipFrames(configuration.automatic_run_settings.index_of_frame_to_start_at);
	}

	mainLoopAction = number_of_frames_to_process_after_launch ? PROCESS_N_FRAMES : PROCESS_PAUSED;
	outImageType[0] = this->freeviewActive ? this->colourModes_freeview[this->currentColourMode].type
	                                       : this->colourModes_main[this->currentColourMode].type;

	if (configuration.telemetry_settings.record_reconstruction_video) {
		this->reconstructionVideoWriter = new FFMPEGWriter();
	}

	if (load_volume_before_automatic_run) {
		if(logger->NeedsFramewiseOutputFolder()){
			logger->SetOutputDirectory(
					this->GenerateCurrentFrameOutputDirectory());
		}
		mainEngine->LoadFromFile();
		SkipFrames(1);
	}
	logger->SetShutdownRequestedFlagLocation(
			&this->shutdownRequested);
	printf("initialised.\n");
}

void UIEngine_BPO::SaveScreenshot(const char* filename) const {
	ITMUChar4Image screenshot(GetWindowSize(), true, false);
	GetScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine_BPO::GetScreenshot(ITMUChar4Image* dest) const {
	glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine_BPO::SkipFrames(int numberOfFramesToSkip) {
	for (int iFrame = 0; iFrame < numberOfFramesToSkip && imageSource->hasMoreImages(); iFrame++) {
		imageSource->getImages(inputRGBImage, inputRawDepthImage);
	}
	this->startedProcessingFromFrameIx += numberOfFramesToSkip;
}


void UIEngine_BPO::ProcessFrame() {
	if (logger->IsRecording3DSceneAndWarpProgression()) {
		std::cout << yellow << "***" << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex()
		          << " (WITH RECORDING 3D SCENES ON)" << yellow << "***" << reset << std::endl;
	} else {
		std::cout << yellow << "***" << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() << yellow << "***"
		          << reset << std::endl;
	}

	if (!imageSource->hasMoreImages()) return;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != nullptr) {
		if (!imuSource->hasMoreMeasurements()) return;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}
	if(logger->NeedsFramewiseOutputFolder()){
		logger->SetOutputDirectory(
				this->GenerateCurrentFrameOutputDirectory());
	}
	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (imuSource != nullptr)
		this->trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif
	sdkStopTimer(&timer_instant);
	sdkStopTimer(&timer_average);

	//processedTime = sdkGetTimerValue(&timer_instant);
	processedTime = sdkGetAverageTimerValue(&timer_average);

	RecordCurrentReconstructionFrameToVideo();

	currentFrameNo++;
}

int UIEngine_BPO::GetCurrentFrameIndex() const {
	return startedProcessingFromFrameIx + currentFrameNo;
}

void UIEngine_BPO::Run() { glutMainLoop(); }


//TODO: should just be in the destructor and triggered when the object goes out of scope -Greg (GitHub:Algomorph)
void UIEngine_BPO::Shutdown() {
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete rgbVideoWriter;
	delete depthVideoWriter;
	delete reconstructionVideoWriter;

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete saveImage;
}


std::string UIEngine_BPO::GenerateNextFrameOutputPath() const {
	fs::path path(std::string(this->output_path) + "/Frame_" + std::to_string(GetCurrentFrameIndex() + 1));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

std::string UIEngine_BPO::GenerateCurrentFrameOutputDirectory() const {
	fs::path path(std::string(this->output_path) + "/Frame_" + std::to_string(GetCurrentFrameIndex()));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

//TODO: Group all recording & make it toggleable with a single keystroke / command flag
void UIEngine_BPO::RecordCurrentReconstructionFrameToVideo() {
	if ((reconstructionVideoWriter != nullptr)) {
		mainEngine->GetImage(outImage[0], outImageType[0], &this->freeviewPose, &freeviewIntrinsics);
		if (outImage[0]->noDims.x != 0) {
			if (!reconstructionVideoWriter->isOpen())
				reconstructionVideoWriter->open((std::string(this->output_path) + "/out_reconstruction.avi").c_str(),
				                                outImage[0]->noDims.x, outImage[0]->noDims.y,
				                                false, 30);
			//TODO This image saving/reading/saving is a production hack -Greg (GitHub:Algomorph)
			//TODO move to a separate function and apply to all recorded video
			//TODO write alternative without OpenCV dependency
#ifdef WITH_OPENCV
			std::string fileName = (std::string(this->output_path) + "/out_reconstruction.png");
			SaveImageToFile(outImage[0], fileName.c_str());
			cv::Mat img = cv::imread(fileName, cv::IMREAD_UNCHANGED);
			cv::putText(img, std::to_string(GetCurrentFrameIndex()), cv::Size(10, 50), cv::FONT_HERSHEY_SIMPLEX,
			            1, cv::Scalar(128, 255, 128), 1, cv::LINE_AA);
			cv::imwrite(fileName, img);
			ITMUChar4Image* imageWithText = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);
			ReadImageFromFile(imageWithText, fileName.c_str());
			reconstructionVideoWriter->writeFrame(imageWithText);
			delete imageWithText;
#else
			reconstructionVideoWriter->writeFrame(outImage[0]);
#endif
		}
	}
}

void UIEngine_BPO::RecordDepthAndRGBInputToVideo() {
	if ((rgbVideoWriter != nullptr) && (inputRGBImage->noDims.x != 0)) {
		if (!rgbVideoWriter->isOpen())
			rgbVideoWriter->open((std::string(this->output_path) + "/out_rgb.avi").c_str(),
			                     inputRGBImage->noDims.x, inputRGBImage->noDims.y, false, 30);
		rgbVideoWriter->writeFrame(inputRGBImage);
	}
	if ((depthVideoWriter != nullptr) && (inputRawDepthImage->noDims.x != 0)) {
		if (!depthVideoWriter->isOpen())
			depthVideoWriter->open((std::string(this->output_path) + "/out_depth.avi").c_str(),
			                       inputRawDepthImage->noDims.x, inputRawDepthImage->noDims.y, true, 30);
		depthVideoWriter->writeFrame(inputRawDepthImage);
	}
}

void UIEngine_BPO::RecordDepthAndRGBInputToImages() {
	if (isRecordingImages) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", output_path.c_str(), currentFrameNo);
		SaveImageToFile(inputRawDepthImage, str);

		if (inputRGBImage->noDims != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", output_path.c_str(), currentFrameNo);
			SaveImageToFile(inputRGBImage, str);
		}
	}
}

void UIEngine_BPO::PrintProcessingFrameHeader() const {
	std::cout << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() + 1;
	if (logger->IsRecording3DSceneAndWarpProgression()) {
		std::cout << " [3D SCENE AND WARP UPDATE RECORDING: ON]";
	}
	if (logger->IsRecordingScene2DSlicesWithUpdates()) {
		std::cout << " [2D SCENE SLICE & WARP UPDATE RECORDING: ON]";
	}
	std::cout << reset << std::endl;
}

