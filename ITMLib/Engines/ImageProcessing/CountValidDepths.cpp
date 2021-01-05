//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/5/21.
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
#include "CountValidDepths.h"

namespace ITMLib::internal{

template<>
int CountValidDepths<MEMORYDEVICE_CPU>(const FloatImage& image_in){
	int valid_point_count = 0;
	const float *image_data = image_in.GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < image_in.size(); ++i) if (image_data[i] > 0.0) valid_point_count++;

	return valid_point_count;
};

} // namespace ITMLib::internal