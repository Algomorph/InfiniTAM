//  ================================================================
//  Created by Gregory Kramida on 6/20/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include <array>

// local
#include "SceneSliceVisualizer3DCommon.h"

namespace ITMLib {
namespace Viz {

//** public **
const std::array<double, 4> canonicalPositiveTruncatedVoxelColor =
		{0.8352941176, 0.8980392157, 0.9607843137, 1.0};
const std::array<double, 4> canonicalPositiveNonTruncatedVoxelColor =
		{0.6588235294, 0.7411764706, 0.9176470588, 1.0};
const std::array<double, 4> canonicalNegativeNonTruncatedVoxelColor =
		{0.1764705882, 0.4980392157, 0.8196078431, 1.0};
const std::array<double, 4> canonicalNegativeTruncatedVoxelColor =
		{0.1058823529, 0.2980392157, 0.4901960784, 1.0};
const std::array<double, 4> canonicalUnknownVoxelColor =
		{0.0352941176, 0.0980392157, 0.1607843137, 1.0};

const std::array<double, 4> canonicalNegativeInterestVoxelColor = {0.690, 0.878, 0.902, 1.0};
const std::array<double, 4> canonicalPositiveInterestVoxelColor = {0.000, 1.000, 1.000, 1.0};

const std::array<double, 4> highlightVoxelColor = {1.000, 0.647, 0.000, 1.0};
const std::array<double, 3> canonicalHashBlockEdgeColor = {0.286, 0.623, 0.854};


const std::array<double, 4> livePositiveTruncatedVoxelColor =
		{0.8352941176, 0.9607843137, 0.8666666667, 1.0};
const std::array<double, 4> livePositiveNonTruncatedVoxelColor =
		{0.5137254902, 1, 0.6078431373, 1.0};
const std::array<double, 4> liveNegativeNonTruncatedVoxelColor =
		{0.1921568627, 0.8039215686, 0.3450980392, 1.0};
const std::array<double, 4> liveNegativeTruncatedVoxelColor =
		{0.1137254902, 0.4823529412, 0.2078431373, 1.0};
const std::array<double, 4> liveUnknownVoxelColor =
		{0.0352941176, 0.1607843137, 0.0666666667, 1.0};

const std::array<double, 3> liveHashBlockEdgeColor = {0.537, 0.819, 0.631};
//** private **

const std::array<std::array<double, 4>, 4> backgroundColors = {{{0.96, 0.96, 0.98, 1.00},  // beige
		                                                               {0.09, 0.07, 0.05, 1.00},  // black raspberry
		                                                               {0.59, 0.44, 0.09, 1.00},  // bristle brown
		                                                               {0.57, 0.64, 0.69, 1.0}}}; // cadet grey
}
}//namespace ITMLib