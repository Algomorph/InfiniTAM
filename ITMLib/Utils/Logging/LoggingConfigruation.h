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



namespace ITMLib {
namespace logging {
//TODO: rework definitions of constants to the CPP file
//static const log4cplus::LogLevel FOCUS_SPOTS_LOG_LEVEL = 1;
//static const log4cplus::LogLevel PER_ITERATION_LOG_LEVEL = 2;
//static const log4cplus::LogLevel PER_FRAME_LOG_LEVEL = 3;
//static const log4cplus::LogLevel TOP_LOG_LEVEL = 3;
//
//#define LOG4CPLUS_FOCUS_SPOTS(logger, logEvent)                            \
//    if(logger.isEnabledFor(ITMLib::logging::FOCUS_SPOTS_LOG_LEVEL)) {                       \
//        log4cplus::tostringstream _log4cplus_buf;                       \
//        _log4cplus_buf << logEvent;                                     \
//	logger.forcedLog(ITMLib::logging::FOCUS_SPOTS_LOG_LEVEL, _log4cplus_buf.str(), __FILE__, __LINE__); \
//    }


void initialize_logging();


}//namespace logging
}//namespace ITMLib


