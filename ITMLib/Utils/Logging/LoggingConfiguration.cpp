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

//local
#include "LoggingConfigruation.h"
#include "../Configuration.h"

using namespace log4cplus;

namespace ITMLib {
namespace logging {






void initialize_logging() {
	log4cplus::initialize();
	//TODO: add list_enumerators function in SerializationDetails.h within the SERIALIZABLE_ENUM_DEFN_IMPL macro that
	// produces a list of strings, so you don't have to enter them manually, like here
	log4cplus::getLogLevelManager().pushLogLevel(FOCUS_SPOTS_LOG_LEVEL, LOG4CPLUS_TEXT("") );
}

}//namespace logging
}//namespace ITMLib
