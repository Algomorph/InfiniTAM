//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/3/21.
//  Copyright (c) 2021 Gregory Kramida
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

// === ORUtils ===
#include "../../../ORUtils/CrossPlatformMacros.h"

// === local ===
#include "Shared/LowLevelEngine_Shared.h"

namespace ITMLib {

template<MemoryDeviceType TMemoryDeviceType>
struct ConvertColorToIntensityFunctor {
private: // instance variables
	const Vector2i dimensions;
public: // instance functions
	explicit ConvertColorToIntensityFunctor(const Vector2i& dimensions) :
	                                        dimensions(dimensions){}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(float& pixel_value_out, const Vector4u& pixel_value_in, int x, int y) {
		// typical NTSC/PAL standard coefficients, see https://en.wikipedia.org/wiki/Grayscale
		pixel_value_out = (0.299f * pixel_value_in.r + 0.587f * pixel_value_in.g + 0.114f * pixel_value_in.b) / 255.f;
	}
};

} // namespace ITMLib