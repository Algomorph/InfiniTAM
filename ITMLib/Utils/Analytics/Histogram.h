//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/1/20.
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

//string
#include <string>
#include <utility>
#include <iomanip>

//local
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {

class Histogram {
private: // instance variables
	std::string name;
	ORUtils::MemoryBlock<unsigned int> histogram_bins;
	unsigned long int unit_count;
public:
	Histogram() : name(""), histogram_bins(), unit_count(0){};
	Histogram(std::string name, unsigned int bin_count, unsigned long int unit_count, MemoryDeviceType memory_device_type) :
			name(std::move(name)),
			histogram_bins(bin_count, memory_device_type),
			unit_count(unit_count) {
		histogram_bins.Clear();
	}

	ORUtils::MemoryBlock<unsigned int>::size_type GetBinCount() {
		return histogram_bins.size();
	}

	unsigned int* GetBinData() {
		return histogram_bins.GetData();
	}

	unsigned long int GetUnitCount() {
		return unit_count;
	}

	std::string GetName() {
		return name;
	}

	friend std::ostream& operator<<(std::ostream& ostream, const Histogram& histogram) {
		const ORUtils::MemoryBlock<unsigned int>* warp_update_bins_to_read;
		if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
			warp_update_bins_to_read = new ORUtils::MemoryBlock<unsigned int>(histogram.histogram_bins.size(), MEMORYDEVICE_CPU);
		} else {
			warp_update_bins_to_read = &histogram.histogram_bins;
		}
		const unsigned int* bin_data = warp_update_bins_to_read->GetData(MEMORYDEVICE_CPU);

		ostream << histogram.name << ":" << std::endl;
		auto unit_count_float = static_cast<float>(histogram.unit_count);

		for (int i_bin = 0; i_bin < histogram.histogram_bins.size(); i_bin++) {
			ostream << std::setw(7) << std::setfill(' ') << std::setprecision(3)
			        << 100.0f * static_cast<float>(bin_data[i_bin]) / unit_count_float << "%";
		}
		if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
			delete warp_update_bins_to_read;
		}
		return ostream;
	}

	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& wrapper, const Histogram& histogram) {
		const ORUtils::MemoryBlock<unsigned int>* warp_update_bins_to_read;
		if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
			warp_update_bins_to_read = new ORUtils::MemoryBlock<unsigned int>(histogram.histogram_bins.size(), MEMORYDEVICE_CPU);
		} else {
			warp_update_bins_to_read = &histogram.histogram_bins;
		}
		const unsigned int* bin_data = warp_update_bins_to_read->GetData(MEMORYDEVICE_CPU);

		wrapper.OStream().write(reinterpret_cast<const char*>(histogram.histogram_bins.size()),
		                        sizeof(ORUtils::MemoryBlock<unsigned int>::size_type));
		for (int i_bin = 0; i_bin < histogram.histogram_bins.size(); i_bin++) {
			wrapper.OStream().write(reinterpret_cast<const char*>(bin_data + i_bin), sizeof(unsigned int));
		}

		if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
			delete warp_update_bins_to_read;
		}
		return wrapper;
	}
};


} // namespace ITMLib