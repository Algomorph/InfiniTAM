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
#pragma once

#include <memory>
#include <ostream>

#include "PlatformIndependence.h"

namespace ORUtils {
class OStreamWrapper {
public:
	OStreamWrapper() : compression_enabled(false), file(nullptr) {};

	explicit OStreamWrapper(const std::string& path, bool use_compression = false);

	bool operator!() {
		return file == nullptr || !file->good();
	}

	std::ostream& OStream() {
		return *file;
	}

	const bool compression_enabled;
private:
	std::unique_ptr<std::ostream> file;
	std::unique_ptr<std::ostream> buffer_file;

};
} // namespace ORUtils