//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/24/20.
//  Copyright (c) $YEAR-2020 Gregory Kramida
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

#include <cstdlib>
#include <cstring>

//log4cplus
#include <log4cplus/logger.h>

#include "LoggingCUDAWrapper.h"

namespace l4c = log4cplus;

namespace ITMLib::logging::CUDA::log4cplus {

Logger::Logger() : logger(new l4c::Logger(l4c::Logger::getRoot())) {}

Logger::~Logger() { delete reinterpret_cast<l4c::Logger*>(this->logger);}

bool Logger::isEnabledFor(LogLevel level) const {
	return reinterpret_cast<l4c::Logger*>(this->logger)->isEnabledFor(level);
}

void Logger::forcedLog(LogLevel ll, const std::string& message, const char* file, int line) {
	reinterpret_cast<l4c::Logger*>(this->logger)->forcedLog(ll, message, file, line);
}

Logger Logger::getRoot() {
	return Logger();
}

} // namespace ITMLib



