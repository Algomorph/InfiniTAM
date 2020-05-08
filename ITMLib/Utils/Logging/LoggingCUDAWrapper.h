//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/24/20.
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

#include <iosfwd>

namespace ITMLib{
namespace logging{
namespace CUDA{

namespace log4cplus{

#if defined (UNICODE)
typedef wchar_t tchar;

#else
typedef char tchar;

#endif
//TODO: revise to use a shared pointer to the root logger somehow, to enable multiple instances.
// The pointer has to use some dummy type, i.e. cannot use log4cpp headers directly. Raw pointers don't work well with log4cpp's internal pointers.

typedef int LogLevel;
typedef std::basic_ostringstream<tchar> tostringstream;
class Logger{
public:
	static Logger getRoot();

	Logger();
	~Logger();
	bool isEnabledFor(LogLevel level) const;
	void forcedLog(LogLevel ll, const std::string& message,
	               const char* file,
	               int line);
private:
	void* logger;
};

} // namespace log4cplus



} // namespace CUDA
} // namespace logging
} // namespace ITMLib

