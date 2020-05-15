// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../ITMLib/Engines/Main/MainEngine.h"
#include "../../ITMLib/Utils/Configuration/Configuration.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class CLIEngine
		{

			InputSource::ImageSourceEngine *image_source;
			InputSource::IMUSourceEngine *imu_source;

			ITMLib::MainEngine *main_engine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

		private:
			UChar4Image *input_RGB_image; ShortImage *input_raw_depth_image;
			ITMLib::IMUMeasurement *input_IMU_measurement;

			int current_frame_index;
			int number_of_frames_to_process_after_launch = 0;
			bool exit_after_automatic_run = false;
			bool save_after_automatic_run = false;
			int start_frame_index = 0;

			std::string output_path;

			void SkipFrames(int number_of_frames_to_skip);
			std::string GenerateCurrentFrameOutputPath() const;
			std::string GeneratePreviousFrameOutputPath() const;
		public:
			static CLIEngine* Instance() {
				static CLIEngine instance;
				return &instance;
			}

			float processedTime;

			void Initialise(InputSource::ImageSourceEngine *image_source, InputSource::IMUSourceEngine *imu_source, ITMLib::MainEngine *main_engine,
			                MemoryDeviceType device_type);
			void Shutdown();

			void Run();
			bool ProcessFrame();
		};
	}
}
