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
//log4cplus
#include <log4cplus/consoleappender.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/initializer.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/logger.h>

//boost
#include <boost/filesystem.hpp>

//local
#include "Logging.h"
#include "../Configuration/Configuration.h"
#include "../FileIO/RecordHandling.h"


using namespace log4cplus;
namespace fs = boost::filesystem;

namespace ITMLib::logging {

static bool logging_initialized = false;

void InitializeLogging() {
	if(logging_initialized){
		return;
	}else{
		logging_initialized = true;
	}

	log4cplus::initialize();
#define HAVE_CONSOLE_SUPPORT_FOR_256_COLORS
#ifdef HAVE_CONSOLE_SUPPORT_FOR_256_COLORS
	log4cplus::getLogLevelManager().pushLogLevel(FOCUS_SPOTS_LOG_LEVEL, LOG4CPLUS_TEXT("\033[38;5;23mFOCUS_SPOTS\033[0m"));
	log4cplus::getLogLevelManager().pushLogLevel(PER_ITERATION_LOG_LEVEL, LOG4CPLUS_TEXT("\033[38;5;64mPER_ITERATION\033[0m"));
	log4cplus::getLogLevelManager().pushLogLevel(PER_FRAME_LOG_LEVEL, LOG4CPLUS_TEXT("\033[38;5;67mPER_FRAME\033[0m"));
	log4cplus::getLogLevelManager().pushLogLevel(TOP_LOG_LEVEL, LOG4CPLUS_TEXT("\033[38;5;78mTOP_LEVEL\033[0m"));
#else
	log4cplus::getLogLevelManager().pushLogLevel(FOCUS_SPOTS_LOG_LEVEL, LOG4CPLUS_TEXT("FOCUS_SPOTS"));
	log4cplus::getLogLevelManager().pushLogLevel(PER_ITERATION_LOG_LEVEL, LOG4CPLUS_TEXT("PER_ITERATION"));
	log4cplus::getLogLevelManager().pushLogLevel(PER_FRAME_LOG_LEVEL, LOG4CPLUS_TEXT("PER_FRAME"));
	log4cplus::getLogLevelManager().pushLogLevel(TOP_LOG_LEVEL, LOG4CPLUS_TEXT("TOP_LEVEL"));
#endif

	auto root = log4cplus::Logger::getRoot();

	switch (configuration::Get().logging_settings.verbosity_level) {
		case VERBOSITY_SILENT:
			root.setLogLevel(OFF_LOG_LEVEL);
			break;
		case VERBOSITY_FATAL:
			root.setLogLevel(FATAL_LOG_LEVEL);
			break;
		case VERBOSITY_ERROR:
			root.setLogLevel(ERROR_LOG_LEVEL);
			break;
		default:
		case VERBOSITY_WARNING:
			root.setLogLevel(WARN_LOG_LEVEL);
			break;
		case VERBOSITY_INFO:
			root.setLogLevel(INFO_LOG_LEVEL);
			break;
		case VERBOSITY_TOP_LEVEL:
			root.setLogLevel(TOP_LOG_LEVEL);
			break;
		case VERBOSITY_PER_FRAME:
			root.setLogLevel(PER_FRAME_LOG_LEVEL);
			break;
		case VERBOSITY_PER_ITERATION:
			root.setLogLevel(PER_ITERATION_LOG_LEVEL);
			break;
		case VERBOSITY_FOCUS_SPOTS:
			root.setLogLevel(FOCUS_SPOTS_LOG_LEVEL);
			break;
		case VERBOSITY_DEBUG:
			root.setLogLevel(DEBUG_LOG_LEVEL);
			break;
	}

	if (configuration::Get().logging_settings.log_to_stdout) {
		log4cplus::SharedAppenderPtr console_appender(new log4cplus::ConsoleAppender(false, true));
		root.addAppender(console_appender);
	}

	if (configuration::Get().logging_settings.log_to_disk) {
		std::string log_path = (fs::path(configuration::Get().paths.output_path) / "log.ans").string();
		ArchivePossibleExistingRecords(log_path, "older_logs");
		log4cplus::SharedFileAppenderPtr file_appender(new RollingFileAppender(
				LOG4CPLUS_TEXT(log_path), 50 * 1024 * 1024, 5, false, true));
		root.addAppender(SharedAppenderPtr(file_appender.get()));
	}


}

Logger GetLogger() {
	return Logger::getRoot();
}

}//namespace ITMLib
