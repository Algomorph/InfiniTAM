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

#include "ImageProcessingEngineInterface.h"

namespace ITMLib {

template<MemoryDeviceType TMemoryDeviceType>
class ImageProcessingEngine : public ImageProcessingEngineInterface {
public: // instance functions
	ImageProcessingEngine();
	~ImageProcessingEngine() override;

	void ConvertColorToIntensity(FloatImage& image_out, const UChar4Image& image_in) const override;

	void FilterIntensity(FloatImage& image_out, const FloatImage& image_in) const override;

	void FilterSubsample(UChar4Image& image_out, const UChar4Image& image_in) const override;
	void FilterSubsample(FloatImage& image_out, const FloatImage& image_in) const override;
	void FilterSubsampleWithHoles(FloatImage& image_out, const FloatImage& image_in) const override;
	void FilterSubsampleWithHoles(Float4Image& image_out, const Float4Image& image_in) const override;

	void GradientX(Short4Image& grad_out, const UChar4Image& image_in) const override;
	void GradientY(Short4Image& grad_out, const UChar4Image& image_in) const override;
	void GradientXY(Float2Image& grad_out, const FloatImage& image_in) const override;

	int CountValidDepths(const FloatImage& image_in) const override;

};


} // namespace ITMLib
