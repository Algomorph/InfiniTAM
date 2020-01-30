// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../ITMLib/Engines/Main/MainEngine.h"
#include "../../ITMLib/Utils/Configuration.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class CLIEngine
		{

			InputSource::ImageSourceEngine *imageSource;
			InputSource::IMUSourceEngine *imuSource;

			ITMLib::MainEngine *mainEngine;

			StopWatchInterface *timer_instant;
			StopWatchInterface *timer_average;

		private:
			ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
			ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

			int currentFrameNo;
		public:
			static CLIEngine* Instance() {
				static CLIEngine instance;
				return &instance;
			}

			float processedTime;

			void Initialise(InputSource::ImageSourceEngine *imageSource, InputSource::IMUSourceEngine *imuSource, ITMLib::MainEngine *mainEngine,
				MemoryDeviceType deviceType);
			void Shutdown();

			void Run();
			bool ProcessFrame();
		};
	}
}
