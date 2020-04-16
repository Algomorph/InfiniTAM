//  ================================================================
//  Created by Gregory Kramida on 6/15/18.
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


#include <iostream>
#include "CardinalAxesAndPlanes.h"


namespace ITMLib{

std::istream& operator >> (std::istream& in, ITMLib::Plane& plane)
{
	std::string token;
	char c[3];
	in >> c;
	token = c;
	if (token == "yz" || token == "YZ")
		plane = Plane::PLANE_YZ;
	else if (token == "xz" || token == "XZ")
		plane = Plane::PLANE_XZ;
	else if (token == "xy" || token == "XY")
		plane = Plane::PLANE_XY;
	else
		in.setstate(std::ios_base::failbit);
	return in;
}
}// end namespace ITMLib
