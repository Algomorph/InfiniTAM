// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ImageProcessingEngineInterface.h"

namespace ITMLib
{
	class LowLevelEngine_CPU : public ImageProcessingEngineInterface
	{
	public:
		void ConvertColorToIntensity(FloatImage& image_out, const UChar4Image& image_in) const;

		void FilterIntensity(FloatImage& image_out, const FloatImage& image_in) const;

		void FilterSubsample(UChar4Image& image_out, const UChar4Image& image_in) const;
		void FilterSubsample(FloatImage& image_out, const FloatImage& image_in) const;
		void FilterSubsampleWithHoles(FloatImage& image_out, const FloatImage& image_in) const;
		void FilterSubsampleWithHoles(Float4Image& image_out, const Float4Image& image_in) const;

		void GradientX(Short4Image& grad_out, const UChar4Image& image_in) const;
		void GradientY(Short4Image& grad_out, const UChar4Image& image_in) const;
		void GradientXY(Float2Image& grad_out, const FloatImage& image_in) const;

		int CountValidDepths(const FloatImage& image_in) const;

		LowLevelEngine_CPU();
		~LowLevelEngine_CPU();
	};
}
