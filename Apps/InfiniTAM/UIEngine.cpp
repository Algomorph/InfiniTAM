#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"


#include "UIEngine.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifndef FREEGLUT_STATIC

#include <GL/glut.h>

#endif
#if defined(FREEGLUT) || defined(FREEGLUT_STATIC)

#include <GL/freeglut.h>

#endif
#endif

//ITMLib
#include "../../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/ImageCombination.h"
#include "../../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../../ITMLib/Utils/Logging/ConsolePrintColors.h"
#include "../../ITMLib/Utils/Telemetry/TelemetryUtilities.h"
#include "../../ITMLib/Utils/FileIO/RecordHandling.h"


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
 * \param configuration remaining configuration settings already pre-assessed from command line or configuration file
 */
void UIEngine::Initialize(int& argc, char** argv,
                          InputSource::ImageSourceEngine* imageSource,
                          InputSource::IMUSourceEngine* imuSource,
                          ITMLib::FusionAlgorithm* main_engine,
                          const configuration::Configuration& configuration) {

	//TODO: just use automatic_run_settings as member directly instead of copying stuff over.
	automatic_run_settings = ExtractSerializableStructFromPtreeIfPresent<AutomaticRunSettings>(
			configuration.source_tree, AutomaticRunSettings::default_parse_path, configuration.origin);
	this->freeview_active = true;
	this->integration_active = true;
	this->current_colour_mode = 0;

	this->colourModes_main.emplace_back("shaded greyscale", FusionAlgorithm::InfiniTAM_IMAGE_SCENERAYCAST);
	this->colourModes_main.emplace_back("integrated colours", FusionAlgorithm::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
	this->colourModes_main.emplace_back("surface normals", FusionAlgorithm::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL);
	this->colourModes_main.emplace_back("confidence", FusionAlgorithm::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE);

	this->colourModes_freeview.emplace_back("canonical", FusionAlgorithm::InfiniTAM_IMAGE_FREECAMERA_CANONICAL);
	this->colourModes_freeview.emplace_back("shaded greyscale", FusionAlgorithm::InfiniTAM_IMAGE_FREECAMERA_SHADED);
	this->colourModes_freeview.emplace_back("integrated colours",
	                                        FusionAlgorithm::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME);
	this->colourModes_freeview.emplace_back("surface normals",
	                                        FusionAlgorithm::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL);
	this->colourModes_freeview.emplace_back("confidence",
	                                        FusionAlgorithm::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE);

	this->image_source_engine = imageSource;
	this->imu_source_engine = imuSource;
	this->main_engine = main_engine;
	this->output_path = configuration.paths.output_path;

	int text_height = 60; // Height of text area, 2 lines

	window_size.x = (int) (1.5f * (float) (imageSource->GetDepthImageSize().x));
	window_size.y = imageSource->GetDepthImageSize().y + text_height;
	float h1 = static_cast<float>(text_height) / (float) window_size.y, h2 = (1.f + h1) / 2;
	window_corners[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	window_corners[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	window_corners[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->image_recording_enabled = false;
	this->processed_frame_count = 0;
	this->RGB_video_writer = nullptr;
	this->depth_video_writer = nullptr;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(window_size.x, window_size.y);
	glutCreateWindow("InfiniTAM");
	glGenTextures(NUM_WIN, textureId);

	glutDisplayFunc(UIEngine::GlutDisplayFunction);
	glutKeyboardUpFunc(UIEngine::GlutKeyUpFunction);
	glutMouseFunc(UIEngine::GlutMouseButtonFunction);
	glutMotionFunc(UIEngine::GlutMouseMoveFunction);
	glutIdleFunc(UIEngine::GlutIdleFunction);

#ifdef FREEGLUT
	glutMouseWheelFunc(UIEngine::GlutMouseWheelFunction);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	bool allocate_GPU = configuration.device_type == MEMORYDEVICE_CUDA;

	for (auto& output_image : output_images) {
		output_image = new UChar4Image(imageSource->GetDepthImageSize(), true, allocate_GPU);
	}

	input_RGB_image = new UChar4Image(imageSource->GetRGBImageSize(), true, allocate_GPU);
	input_raw_depth_image = new ShortImage(imageSource->GetDepthImageSize(), true, allocate_GPU);
	input_IMU_measurement = new IMUMeasurement();

	image_to_save = new UChar4Image(imageSource->GetDepthImageSize(), true, false);


	output_image_types[1] = FusionAlgorithm::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	output_image_types[2] = FusionAlgorithm::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (input_RGB_image->dimensions == Vector2i(0, 0)) output_image_types[2] = FusionAlgorithm::InfiniTAM_IMAGE_UNKNOWN;

	current_mouse_operation = IDLE;
	mouse_warped = false;
	needs_refresh = false;

	current_frame_processing_time = 0.0f;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);
	current_frame_index = 0;
	if (automatic_run_settings.index_of_frame_to_start_at > 0) {
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(),
		                    "Skipping the first " << automatic_run_settings.index_of_frame_to_start_at << " frames.");
		SkipFrames(automatic_run_settings.index_of_frame_to_start_at);
	}

	main_loop_action = automatic_run_settings.index_of_frame_to_end_before > 0 ? PROCESS_N_FRAMES : PROCESS_PAUSED;
	output_image_types[0] = this->freeview_active ? this->colourModes_freeview[this->current_colour_mode].type
	                                              : this->colourModes_main[this->current_colour_mode].type;

	if (configuration.record_reconstruction_video) {
		this->reconstruction_video_writer = new FFMPEGWriter();
		if (configuration.record_inputs_in_reconstruction_video) {
			this->add_input_to_reconstruction_video = true;
			image_processing_engine = ImageProcessingEngineFactory::Build(MEMORYDEVICE_CPU);
		}
	}

	if (automatic_run_settings.load_volume_and_camera_matrix_before_processing) {
		std::string frame_path = this->GenerateCurrentFrameOutputPath();
		main_engine->LoadFromFile(frame_path);
		SkipFrames(1);
	}
	LOG4CPLUS_TOP_LEVEL(logging::GetLogger(), "initialised.");
}

void UIEngine::SaveScreenshot(const char* filename) const {
	UChar4Image screenshot(GetWindowSize(), true, false);
	GetScreenshot(&screenshot);
	ORUtils::SaveImageToFile(screenshot, filename, true);
}

void UIEngine::GetScreenshot(UChar4Image* dest) const {
	glReadPixels(0, 0, dest->dimensions.x, dest->dimensions.y, GL_RGBA, GL_UNSIGNED_BYTE,
	             dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine::SkipFrames(int number_of_frames_to_skip) {
	for (int i_frame = 0; i_frame < number_of_frames_to_skip && image_source_engine->HasMoreImages(); i_frame++) {
		image_source_engine->GetImages(*input_RGB_image, *input_raw_depth_image);
	}
	this->current_frame_index += number_of_frames_to_skip;
}


void UIEngine::ProcessFrame() {
	LOG4CPLUS_PER_FRAME(logging::GetLogger(),
	                    yellow << "***" << bright_cyan << " PROCESSING FRAME " << current_frame_index << yellow
	                           << " ***" << reset);

	if (!image_source_engine->HasMoreImages()) return;
	image_source_engine->GetImages(*input_RGB_image, *input_raw_depth_image);

	if (imu_source_engine != nullptr) {
		if (!imu_source_engine->hasMoreMeasurements()) return;
		else imu_source_engine->getMeasurement(input_IMU_measurement);
	}

	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);

	//actual processing on the main_engine
	if (imu_source_engine != nullptr)
		this->tracking_result = main_engine->ProcessFrame(input_RGB_image, input_raw_depth_image, input_IMU_measurement);
	else tracking_result = main_engine->ProcessFrame(input_RGB_image, input_raw_depth_image);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif
	sdkStopTimer(&timer_instant);
	sdkStopTimer(&timer_average);

	//current_frame_processing_time = sdkGetTimerValue(&timer_instant);
	current_frame_processing_time = sdkGetAverageTimerValue(&timer_average);

	RecordCurrentReconstructionFrameToVideo();

	processed_frame_count++;
}

void UIEngine::Run() { glutMainLoop(); }


//TODO: should just be in the destructor and triggered when the object goes out of scope -Greg (GitHub:Algomorph)
void UIEngine::Shutdown() {
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete RGB_video_writer;
	delete depth_video_writer;
	delete reconstruction_video_writer;

	for (auto& output_image : output_images)
		delete output_image;

	delete input_RGB_image;
	delete input_raw_depth_image;
	delete input_IMU_measurement;

	delete image_to_save;
}


std::string UIEngine::GenerateNextFrameOutputPath() const {
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(current_frame_index + 1);
}

std::string UIEngine::GenerateCurrentFrameOutputPath() const {
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(current_frame_index);
}

std::string UIEngine::GeneratePreviousFrameOutputPath() const {
	return ITMLib::telemetry::CreateAndGetOutputPathForFrame(current_frame_index - 1);
}

/**
 * \brief If the recorded output images refer to a frame other than the current frame, update them with the current images.
 * \details Note: assumes that the main engine has processed the current frame.
 */
void UIEngine::UpdateOutputImages() {
	if (output_image_frame_index != current_frame_index) {
		main_engine->GetImage(output_images[0], output_image_types[0], &this->freeview_pose, &freeview_intrinsics);
		for (int i_output_image = 1; i_output_image < NUM_WIN; i_output_image++) {
			main_engine->GetImage(output_images[i_output_image], output_image_types[i_output_image]);
		}
		output_image_frame_index = current_frame_index;
	}
}

//TODO: Group all recording & make it toggleable with a single keystroke / command flag
void UIEngine::RecordCurrentReconstructionFrameToVideo() {
	if ((reconstruction_video_writer != nullptr)) {
		UpdateOutputImages();
		if (output_images[0]->dimensions.x != 0) {
			if (this->add_input_to_reconstruction_video) {
				assert(output_image_types[1] == FusionAlgorithm::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
				Vector2i input_dims_depth = output_images[1]->dimensions;
				Vector2i reduced_dims_depth(input_dims_depth.x / 2, input_dims_depth.y / 2);
				UChar4Image reduced_depth(reduced_dims_depth, MEMORYDEVICE_CPU);
				image_processing_engine->FilterSubsample(reduced_depth, *output_images[1]);

				assert(output_image_types[2] == FusionAlgorithm::InfiniTAM_IMAGE_ORIGINAL_RGB);
				Vector2i input_dims_RGB = output_images[2]->dimensions;
				Vector2i reduced_dims_RGB(input_dims_RGB.x / 2, input_dims_RGB.y / 2);
				UChar4Image reduced_RGB(reduced_dims_RGB, MEMORYDEVICE_CPU);
				image_processing_engine->FilterSubsample(reduced_RGB, *output_images[2]);

				assert(reduced_dims_RGB.y + reduced_dims_depth.y == output_images[0]->dimensions.y);
				UChar4Image rgb_and_depth = ORUtils::ConcatenateImages(std::vector<std::reference_wrapper<UChar4Image>>{reduced_depth, reduced_RGB},
				                                                       0);
				UChar4Image video_output_image = ORUtils::ConcatenateImages(
						std::vector<std::reference_wrapper<UChar4Image>>{*output_images[0], rgb_and_depth}, 1);
				if (!reconstruction_video_writer->isOpen()) {
					std::string video_path = std::string(this->output_path) + "/out_reconstruction.avi";
					ArchivePossibleExistingRecords(video_path, "older_reconstruction_videos");
					reconstruction_video_writer->open(video_path.c_str(), video_output_image.dimensions.x, video_output_image.dimensions.y,
					                                  false, 30);
				}
				reconstruction_video_writer->writeFrame(&video_output_image);
			} else {
				if (!reconstruction_video_writer->isOpen()) {
					std::string video_path = std::string(this->output_path) + "/out_reconstruction.avi";
					ArchivePossibleExistingRecords(video_path, "older_reconstruction_videos");
					reconstruction_video_writer->open(video_path.c_str(), output_images[0]->dimensions.x,
					                                  output_images[0]->dimensions.y, false, 30);
				}
				reconstruction_video_writer->writeFrame(output_images[0]);
			}
		}
	}
}

void UIEngine::RecordDepthAndRGBInputToVideo() {
	if ((RGB_video_writer != nullptr) && (input_RGB_image->dimensions.x != 0)) {
		if (!RGB_video_writer->isOpen())
			RGB_video_writer->open((std::string(this->output_path) + "/out_rgb.avi").c_str(),
			                       input_RGB_image->dimensions.x, input_RGB_image->dimensions.y, false, 30);
		RGB_video_writer->writeFrame(input_RGB_image);
	}
	if ((depth_video_writer != nullptr) && (input_raw_depth_image->dimensions.x != 0)) {
		if (!depth_video_writer->isOpen())
			depth_video_writer->open((std::string(this->output_path) + "/out_depth.avi").c_str(),
			                         input_raw_depth_image->dimensions.x, input_raw_depth_image->dimensions.y, true, 30);
		depth_video_writer->writeFrame(input_raw_depth_image);
	}
}

void UIEngine::RecordDepthAndRGBInputToImages() {
	if (image_recording_enabled) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", output_path.c_str(), processed_frame_count);
		ORUtils::SaveImageToFile(*input_raw_depth_image, str);

		if (input_RGB_image->dimensions != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", output_path.c_str(), processed_frame_count);
			ORUtils::SaveImageToFile(*input_RGB_image, str);
		}
	}
}

void UIEngine::PrintProcessingFrameHeader() const {
	LOG4CPLUS_PER_FRAME(logging::GetLogger(),
	                    bright_cyan << "PROCESSING FRAME " << current_frame_index + 1 << reset);
}


#pragma clang diagnostic pop