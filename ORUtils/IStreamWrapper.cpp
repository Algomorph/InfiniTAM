//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/23/20.
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
#include <memory>
#include <fstream>

#include <zstr/zstr.hpp>

//local
#include "IStreamWrapper.h"

using namespace ORUtils;

IStreamWrapper::IStreamWrapper(const std::string& path, bool use_compression)
		: file(nullptr),
		  compression_enabled(use_compression) {
	if (use_compression) {
		file = std::make_unique<zstr::ifstream>(path, std::ios::binary | std::ios::in);
	} else {
		file = std::make_unique<std::ifstream>(path, std::ios::binary | std::ios::in);
	}
	if (!file->good()) {
		std::stringstream ss;
		ss << "Could not open file \"" << path << "\" for writing.\n[" __FILE__ ":" TOSTRING(__LINE__) "]";
		throw std::runtime_error(ss.str());
	}
}
