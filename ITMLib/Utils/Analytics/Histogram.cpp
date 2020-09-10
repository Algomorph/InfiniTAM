//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/8/20.
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

#include "Histogram.h"

namespace ITMLib {

std::ostream& operator<<(std::ostream& ostream, const Histogram& histogram) {
	const ORUtils::MemoryBlock<unsigned int>* warp_update_bins_to_read;
	if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
		warp_update_bins_to_read = new ORUtils::MemoryBlock<unsigned int>(histogram.histogram_bins, MEMORYDEVICE_CPU);
	} else {
		warp_update_bins_to_read = &histogram.histogram_bins;
	}
	const unsigned int* bin_data = warp_update_bins_to_read->GetData(MEMORYDEVICE_CPU);
	histogram.unit_count.UpdateHostFromDevice();
	auto unit_count = static_cast<float>(histogram.unit_count.GetData(MEMORYDEVICE_CPU)[0]);

	ostream << histogram.name << ":" << std::endl;

	for (int i_bin = 0; i_bin < histogram.histogram_bins.size(); i_bin++) {
		ostream << std::setw(7) << std::setfill(' ') << std::fixed << std::setprecision(4)
		        << 100.0f * static_cast<float>(bin_data[i_bin]) / unit_count << "%";
	}
	if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
		delete warp_update_bins_to_read;
	}
	return ostream;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& wrapper, const Histogram& histogram) {
	const ORUtils::MemoryBlock<unsigned int>* warp_update_bins_to_read;
	if (histogram.histogram_bins.GetAccessMode() == MEMORYDEVICE_CUDA) {
		warp_update_bins_to_read = new ORUtils::MemoryBlock<unsigned int>(histogram.histogram_bins.size(), MEMORYDEVICE_CPU);
	} else {
		warp_update_bins_to_read = &histogram.histogram_bins;
	}
	const unsigned int* bin_data = warp_update_bins_to_read->GetData(MEMORYDEVICE_CPU);
	histogram.unit_count.UpdateHostFromDevice();
	unsigned long long int element_count = histogram.unit_count.GetData(MEMORYDEVICE_CPU)[0];
	wrapper.OStream().write(reinterpret_cast<const char*>(element_count), sizeof(unsigned long long int));
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

} // namespace ITMLib