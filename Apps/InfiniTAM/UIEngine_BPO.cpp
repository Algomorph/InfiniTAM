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

#include "../../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../../ITMLib/Utils/Logging/LoggingConfigruation.h"
#include "../../ITMLib/Engines/Telemetry/TelemetryRecorder.h"

#ifdef WITH_OPENCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif


//TODO: we should never have to downcast the main engine to some other engine type, architecture needs to be altered
// (potentially by introducting empty method stubs) -Greg (GitHub:Algomorph)

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


namespace bench = ITMLib::benchmarking;


/**
 * \brief Initialize the UIEngine using the specified settings
 * \param argc arguments to the main function
 * \param argv number of arguments to the main function
 * \param imageSource source for images
 * \param imuSource source for IMU data
 * \param main_engine main engine to process the frames
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
                              ITMLib::MainEngine* main_engine,

                              const configuration::Configuration& configuration) {
	this->indexing_method = configuration.indexing_method;

	this->save_after_automatic_run = configuration.automatic_run_settings.save_volumes_after_processing;
	this->exit_after_automatic_run = configuration.automatic_run_settings.exit_after_automatic_processing;

	this->freeview_active = true;
	this->integration_active = true;
	this->current_colour_mode = 0;
	this->number_of_frames_to_process_after_launch = configuration.automatic_run_settings.number_of_frames_to_process;

	this->colourModes_main.emplace_back("shaded greyscale", MainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
	this->colourModes_main.emplace_back("integrated colours", MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
	this->colourModes_main.emplace_back("surface normals", MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL);
	this->colourModes_main.emplace_back("confidence", MainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE);

	this->colourModes_freeview.emplace_back("canonical", MainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL);
	this->colourModes_freeview.emplace_back("shaded greyscale", MainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED);
	this->colourModes_freeview.emplace_back("integrated colours",
	                                        MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME);
	this->colourModes_freeview.emplace_back("surface normals",
	                                        MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL);
	this->colourModes_freeview.emplace_back("confidence",
	                                        MainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE);

	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = main_engine;
	this->output_path = configuration.paths.output_path;

	int textHeight = 60; // Height of text area, 2 lines

	winSize.x = (int) (1.5f * (float) (imageSource->GetDepthImageSize().x));
	winSize.y = imageSource->GetDepthImageSize().y + textHeight;
	float h1 = textHeight / (float) winSize.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->isRecordingImages = false;
	this->processed_frame_count = 0;
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
		outImage[w] = new ITMUChar4Image(imageSource->GetDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(imageSource->GetRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->GetDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new IMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->GetDepthImageSize(), true, false);


	outImageType[1] = MainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = MainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->dimensions == Vector2i(0, 0)) outImageType[2] = MainEngine::InfiniTAM_IMAGE_UNKNOWN;


	auto_interval_frame_start = 0;
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
	outImageType[0] = this->freeview_active ? this->colourModes_freeview[this->current_colour_mode].type
	                                        : this->colourModes_main[this->current_colour_mode].type;

	if (configuration.telemetry_settings.record_reconstruction_video) {
		this->reconstructionVideoWriter = new FFMPEGWriter();
	}

	if (configuration.automatic_run_settings.load_volume_before_processing) {
		std::string frame_path = this->GenerateCurrentFrameOutputPath();
		main_engine->LoadFromFile(frame_path);
		SkipFrames(1);
	}
	printf("initialised.\n");
}

void UIEngine_BPO::SaveScreenshot(const char* filename) const {
	ITMUChar4Image screenshot(GetWindowSize(), true, false);
	GetScreenshot(&screenshot);
	SaveImageToFile(screenshot, filename, true);
}

void UIEngine_BPO::GetScreenshot(ITMUChar4Image* dest) const {
	glReadPixels(0, 0, dest->dimensions.x, dest->dimensions.y, GL_RGBA, GL_UNSIGNED_BYTE,
	             dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine_BPO::SkipFrames(int number_of_frames_to_skip) {
	for (int i_frame = 0; i_frame < number_of_frames_to_skip && imageSource->HasMoreImages(); i_frame++) {
		imageSource->GetImages(*inputRGBImage, *inputRawDepthImage);
	}
	this->start_frame_index += number_of_frames_to_skip;
}


void UIEngine_BPO::ProcessFrame() {
	LOG4CPLUS_TOP_LEVEL(logging::get_logger(),
	                    yellow << "***" << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() << yellow
	                           << "***" << reset);

	if (!imageSource->HasMoreImages()) return;
	imageSource->GetImages(*inputRGBImage, *inputRawDepthImage);

	if (imuSource != nullptr) {
		if (!imuSource->hasMoreMeasurements()) return;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);

	//actual processing on the mainEngine
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

	processed_frame_count++;
}

int UIEngine_BPO::GetCurrentFrameIndex() const {
	return start_frame_index + processed_frame_count;
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
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(GetCurrentFrameIndex() + 1);
}

std::string UIEngine_BPO::GenerateCurrentFrameOutputPath() const {
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(GetCurrentFrameIndex());
}

std::string UIEngine_BPO::GeneratePreviousFrameOutputPath() const {
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(GetCurrentFrameIndex() - 1);
}

//TODO: Group all recording & make it toggleable with a single keystroke / command flag
void UIEngine_BPO::RecordCurrentReconstructionFrameToVideo() {
	if ((reconstructionVideoWriter != nullptr)) {
		mainEngine->GetImage(outImage[0], outImageType[0], &this->freeviewPose, &freeviewIntrinsics);
		if (outImage[0]->dimensions.x != 0) {
			if (!reconstructionVideoWriter->isOpen())
				reconstructionVideoWriter->open((std::string(this->output_path) + "/out_reconstruction.avi").c_str(),
				                                outImage[0]->dimensions.x, outImage[0]->dimensions.y,
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
			ITMUChar4Image* imageWithText = new ITMUChar4Image(image_source->getDepthImageSize(), true, allocateGPU);
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
	if ((rgbVideoWriter != nullptr) && (inputRGBImage->dimensions.x != 0)) {
		if (!rgbVideoWriter->isOpen())
			rgbVideoWriter->open((std::string(this->output_path) + "/out_rgb.avi").c_str(),
			                     inputRGBImage->dimensions.x, inputRGBImage->dimensions.y, false, 30);
		rgbVideoWriter->writeFrame(inputRGBImage);
	}
	if ((depthVideoWriter != nullptr) && (inputRawDepthImage->dimensions.x != 0)) {
		if (!depthVideoWriter->isOpen())
			depthVideoWriter->open((std::string(this->output_path) + "/out_depth.avi").c_str(),
			                       inputRawDepthImage->dimensions.x, inputRawDepthImage->dimensions.y, true, 30);
		depthVideoWriter->writeFrame(inputRawDepthImage);
	}
}

void UIEngine_BPO::RecordDepthAndRGBInputToImages() {
	if (isRecordingImages) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", output_path.c_str(), processed_frame_count);
		SaveImageToFile(*inputRawDepthImage, str);

		if (inputRGBImage->dimensions != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", output_path.c_str(), processed_frame_count);
			SaveImageToFile(*inputRGBImage, str);
		}
	}
}

void UIEngine_BPO::PrintProcessingFrameHeader() const {
	LOG4CPLUS_PER_FRAME(logging::get_logger(),
	                    bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() + 1 << reset);
}

