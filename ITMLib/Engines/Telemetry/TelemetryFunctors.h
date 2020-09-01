//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 8/31/20.
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

//stdlib
#include <iostream>

//local
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../ORUtils/Vector.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib{

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct WarpHistogramFunctor {
	WarpHistogramFunctor(float max_warp_update_length, unsigned int warp_count, int bin_count) :
			max_warp_update_length(max_warp_update_length), warp_count(warp_count), histogram_bin_count(bin_count),
			warp_update_bins(bin_count, TMemoryDeviceType),
			warp_update_bins_device(warp_update_bins.GetData(TMemoryDeviceType)) {
		warp_update_bins.Clear();
	}

	const int histogram_bin_count;
	const unsigned int warp_count;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarp& warp) {
		float warp_update_length = ORUtils::length(warp.warp_update);
		int bin_index = 0;

		if (max_warp_update_length > 0) {
			bin_index = ORUTILS_MIN(histogram_bin_count - 1,
			                        (int) (warp_update_length * histogram_bin_count / max_warp_update_length));
		}
		warp_update_bins_device[bin_index]++;
	}

	void PrintHistogram() {
		ORUtils::MemoryBlock<int>* warp_update_bins_to_read;
		if (TMemoryDeviceType == MEMORYDEVICE_CUDA) {
			warp_update_bins_to_read = new ORUtils::MemoryBlock<int>(warp_update_bins, MEMORYDEVICE_CPU);
		} else {
			warp_update_bins_to_read = &warp_update_bins;
		}
		int* bin_data = warp_update_bins_to_read->GetData(MEMORYDEVICE_CPU);

		std::cout << "Update length histogram: " << std::endl;
		float warp_count_float = static_cast<float>(warp_count);

		for (int i_bin = 0; i_bin < histogram_bin_count; i_bin++) {
			std::cout << std::setw(7) << std::setfill(' ') << std::setprecision(3)
			          << 100.0f * static_cast<float>(bin_data[i_bin]) / warp_count_float << "%";
		}
		std::cout << std::endl;
		if (TMemoryDeviceType == MEMORYDEVICE_CUDA) {
			delete warp_update_bins_to_read;
		}
	}


private:
	const float max_warp_update_length;

	ORUtils::MemoryBlock<int> warp_update_bins;
	int* warp_update_bins_device;
};
} // namespace ITMLib