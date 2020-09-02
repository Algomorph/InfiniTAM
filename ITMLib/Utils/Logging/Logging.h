//  ================================================================
//  Created by Gregory Kramida on 3/4/20.
//  Copyright (c) 2020 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once
#include <log4cplus/logger.h>

namespace ITMLib{
namespace logging {
	//TODO: rework definitions of constants to the CPU file
	const int FOCUS_SPOTS_LOG_LEVEL = 5000;
	const int PER_ITERATION_LOG_LEVEL = 5001;
	const int PER_FRAME_LOG_LEVEL = 5002;
	const int TOP_LOG_LEVEL = 5003;

	#define LOG4CPLUS_FOCUS_SPOTS(logger, logEvent) \
	if(logger.isEnabledFor(ITMLib::logging::FOCUS_SPOTS_LOG_LEVEL)) { \
	std::basic_ostringstream<log4cplus::tchar> _log4cplus_buf; \
	_log4cplus_buf << logEvent; \
	logger.forcedLog(ITMLib::logging::FOCUS_SPOTS_LOG_LEVEL, _log4cplus_buf.str(), __FILE__, __LINE__); \
	}
	#define LOG4CPLUS_PER_ITERATION(logger, logEvent) \
	if(logger.isEnabledFor(ITMLib::logging::PER_ITERATION_LOG_LEVEL)) { \
	std::basic_ostringstream<log4cplus::tchar> _log4cplus_buf; \
	_log4cplus_buf << logEvent; \
	logger.forcedLog(ITMLib::logging::PER_ITERATION_LOG_LEVEL, _log4cplus_buf.str(), __FILE__, __LINE__); \
	}
	#define LOG4CPLUS_PER_FRAME(logger, logEvent) \
	if(logger.isEnabledFor(ITMLib::logging::PER_FRAME_LOG_LEVEL)) { \
	std::basic_ostringstream<log4cplus::tchar> _log4cplus_buf; \
	_log4cplus_buf << logEvent; \
	logger.forcedLog(ITMLib::logging::PER_FRAME_LOG_LEVEL, _log4cplus_buf.str(), __FILE__, __LINE__); \
	}
	#define LOG4CPLUS_TOP_LEVEL(logger, logEvent) \
	if(logger.isEnabledFor(ITMLib::logging::TOP_LOG_LEVEL)) { \
	std::basic_ostringstream<log4cplus::tchar> _log4cplus_buf; \
	_log4cplus_buf << logEvent; \
	logger.forcedLog(ITMLib::logging::TOP_LOG_LEVEL, _log4cplus_buf.str(), __FILE__, __LINE__); \
	}

	void InitializeLogging();

	log4cplus::Logger GetLogger();



}// namespace logging
}// namespace ITMLib


