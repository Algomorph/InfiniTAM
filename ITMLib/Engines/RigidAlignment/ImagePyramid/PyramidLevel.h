//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/13/21.
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

//stdlib
#include <vector>
#include <tuple>
#include <memory>

// ORUtils
#include "../../../../ORUtils/Image.h"
#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../Utils/Math.h"



namespace ITMLib {


template<typename TImage>
class PyramidImage {
public:
	virtual const TImage& GetImage() const = 0;
	virtual TImage& GetImage() = 0;
};

template<typename TImage>
class PyramidBaseImage : public PyramidImage<TImage> {
private:
	const TImage* source_image;
public:
	const TImage& GetImage() const override;
	TImage& GetImage();
};


template<typename TImage, MemoryDeviceType TMemoryDeviceType>
class PyramidLevelImage : public PyramidImage<TImage> {
private:
	TImage image;
public:
	PyramidLevelImage() : image(Vector2i(0), TMemoryDeviceType) {}
	const TImage& GetImage() const override;
	TImage& GetImage();
};

template<bool THasBaseLevel, typename... Args>
struct PyramidLevel{
	std::tuple<Args...> pyramid_images;


};




} // namespace ITMLib