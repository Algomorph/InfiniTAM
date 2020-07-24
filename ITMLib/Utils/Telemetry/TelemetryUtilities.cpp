//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/28/20.
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

//stdlib
#include <filesystem>

//local
#include "TelemetryUtilities.h"
#include "../Configuration/Configuration.h"

namespace fs = std::filesystem;
namespace ITMLib {
namespace telemetry {

static int global_frame_index = -1;

bool GlobalFrameIndexWasInitialized() {
	return global_frame_index > 0;
}

int GetGlobalFrameIndex() {
	if (GlobalFrameIndexWasInitialized()) {
		return global_frame_index;
	} else {
		DIEWITHEXCEPTION_REPORTLOCATION("Argh! The global frame index hasn't been initialized. What a bummer!");
		return 0;
	}
}

void SetGlobalFrameIndex(int frame_index) {
	global_frame_index = frame_index;
}

static inline std::string CreateAndGetOutputPathForFrame_Aux(const std::string& frame_folder_postfix) {
	fs::path path(configuration::Get().paths.output_path + frame_folder_prefix + frame_folder_postfix);
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

static std::string pad_with_leading_zeros(const int value, const unsigned total_length)
{
	std::ostringstream oss;
	oss << std::setw(total_length) << std::setfill('0') << value;
	return oss.str();
}

std::string CreateAndGetOutputPathForFrame(int frame_index) {
	return CreateAndGetOutputPathForFrame_Aux(pad_with_leading_zeros(frame_index, 3));
}

std::string CreateAndGetOutputPathForUnknownFrame() {
	return CreateAndGetOutputPathForFrame_Aux("Unknown");
}

std::string CreateAndGetOutputPathForFrame() {
#ifdef UNKNOWN_FRAME_OUTPUT_ALLOWED
	return GlobalFrameIndexWasInitialized() ? CreateAndGetOutputPathForFrame(global_frame_index)
	                                        : CreateAndGetOutputPathForUnknownFrame();
#else
	return CreateAndGetOutputPathForFrame(GetGlobalFrameIndex());
#endif
}


} // namespace telemetry
} // namespace ITMLib