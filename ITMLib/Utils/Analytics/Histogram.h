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
#include "../../../ORUtils/OStreamWrapper.h"

namespace ITMLib {

class Histogram {
private: // instance variables
	std::string name;
	ORUtils::MemoryBlock<unsigned int> histogram_bins;
	ORUtils::MemoryBlock<unsigned long long int> unit_count;
public: // instance functions
	Histogram() : name(""), histogram_bins(), unit_count(){};
	Histogram(std::string name, unsigned int bin_count, MemoryDeviceType memory_device_type) :
			name(std::move(name)),
			histogram_bins(bin_count, memory_device_type),
			unit_count(1, true, memory_device_type == MEMORYDEVICE_CUDA) {
		histogram_bins.Clear();
		unit_count.GetData(MEMORYDEVICE_CPU)[0] = 0u;
		unit_count.UpdateDeviceFromHost();
	}

	ORUtils::MemoryBlock<unsigned int>::size_type GetBinCount() {
		return histogram_bins.size();
	}

	unsigned int* GetBinData() {
		return histogram_bins.GetData();
	}

	unsigned long int GetUnitCount() {
		unit_count.UpdateHostFromDevice();
		return unit_count.GetData(MEMORYDEVICE_CPU)[0];
	}

	unsigned long long int* GetUnitCountData(){
		return unit_count.GetData(histogram_bins.GetAccessMode());
	}

	std::string GetName() {
		return name;
	}


	friend std::ostream& operator<<(std::ostream& ostream, const Histogram& histogram);

	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& wrapper, const Histogram& histogram);
};


} // namespace ITMLib