// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//stdlib
#include <cstring>
#include <filesystem>

//local
#include "CLIEngine.h"

//ORUtils
#include "../../ORUtils/FileUtils.h"

//ITMLib
#include "../../ITMLib/Utils/Configuration/Configuration.h"
#include "../../ITMLib/Utils/Configuration/AutomaticRunSettings.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace fs = std::filesystem;

void CLIEngine::Initialise(ImageSourceEngine *image_source, IMUSourceEngine *imu_source, FusionAlgorithm *main_engine,
                           MemoryDeviceType device_type)
{
	ITMLib::configuration::Configuration& configuration = ITMLib::configuration::get();
	//TODO: just use automatic_run_settings as member directly instead of copying stuff over.
	AutomaticRunSettings automatic_run_settings = ExtractSerializableStructFromPtreeIfPresent<AutomaticRunSettings>(configuration.source_tree,
	                                                                                                                AutomaticRunSettings::default_parse_path,
	                                                                                                                configuration.origin);
	this->output_path = configuration.paths.output_path;

	this->image_source = image_source;
	this->imu_source = imu_source;
	this->main_engine = main_engine;

	bool allocate_GPU = false;
	if (device_type == MEMORYDEVICE_CUDA) allocate_GPU = true;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif

	input_RGB_image = new UChar4Image(image_source->GetRGBImageSize(), true, allocate_GPU);
	input_raw_depth_image = new ShortImage(image_source->GetDepthImageSize(), true, allocate_GPU);
	input_IMU_measurement = new IMUMeasurement();

	this->current_frame_index = 0;
	if (automatic_run_settings.index_of_frame_to_start_at > 0) {
		printf("Skipping the first %d frames.\n", automatic_run_settings.index_of_frame_to_start_at);
		SkipFrames(automatic_run_settings.index_of_frame_to_start_at);
	}

	this->save_after_automatic_run = automatic_run_settings.save_volumes_and_camera_matrix_after_processing;
	this->index_of_frame_to_end_before = automatic_run_settings.index_of_frame_to_end_before;

	if (automatic_run_settings.load_volume_and_camera_matrix_before_processing) {
		std::string frame_path = this->GenerateCurrentFrameOutputPath();
		printf("Loading volume from %s.\n", frame_path.c_str());
		main_engine->LoadFromFile(frame_path);
		SkipFrames(1);
	}

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);
	sdkResetTimer(&timer_average);

	printf("initialised.\n");
}

bool CLIEngine::ProcessFrame()
{
	if (!image_source->HasMoreImages()) return false;
	image_source->GetImages(*input_RGB_image, *input_raw_depth_image);

	if (imu_source != NULL) {
		if (!imu_source->hasMoreMeasurements()) return false;
		else imu_source->getMeasurement(input_IMU_measurement);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant); sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (imu_source != NULL) main_engine->ProcessFrame(input_RGB_image, input_raw_depth_image, input_IMU_measurement);
	else main_engine->ProcessFrame(input_RGB_image, input_raw_depth_image);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaDeviceSynchronize());
#endif
	sdkStopTimer(&timer_instant); sdkStopTimer(&timer_average);

	float processedTime_inst = sdkGetTimerValue(&timer_instant);
	float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

	printf("frame %i: time %.2f, avg %.2f\n", current_frame_index, processedTime_inst, processedTime_avg);



	return true;
}

void CLIEngine::Run()
{
	while (this->index_of_frame_to_end_before <= 0 || this->current_frame_index < this->index_of_frame_to_end_before) {
		if (!ProcessFrame()) break;
		current_frame_index++;
	}
}

void CLIEngine::Shutdown()
{
	if(this->save_after_automatic_run){
		std::string frame_path = this->GeneratePreviousFrameOutputPath();
		printf("Saving volume to %s.\n", frame_path.c_str());
		main_engine->SaveToFile(frame_path);
	}
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete input_RGB_image;
	delete input_raw_depth_image;
	delete input_IMU_measurement;
}

void CLIEngine::SkipFrames(int number_of_frames_to_skip) {
	for (int i_frame = 0; i_frame < number_of_frames_to_skip && image_source->HasMoreImages(); i_frame++) {
		image_source->GetImages(*input_RGB_image, *input_raw_depth_image);
	}
	this->current_frame_index += number_of_frames_to_skip;
}

std::string CLIEngine::GenerateCurrentFrameOutputPath() const {
	fs::path path(std::string(this->output_path) + "/Frame_" + std::to_string(this->current_frame_index));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

std::string CLIEngine::GeneratePreviousFrameOutputPath() const {
	fs::path path(std::string(this->output_path) + "/Frame_" + std::to_string(this->current_frame_index - 1));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}
