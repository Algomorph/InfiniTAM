// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/LowLevelEngine.h"

namespace ITMLib
{
	class LowLevelEngine_CUDA : public LowLevelEngine
	{
	private:
		int *counterTempData_device, *counterTempData_host;

	public:
		void CopyImage(UChar4Image *image_out, const UChar4Image *image_in) const;
		void CopyImage(FloatImage *image_out, const FloatImage *image_in) const;
		void CopyImage(Float4Image *image_out, const Float4Image *image_in) const;

		void ConvertColourToIntensity(FloatImage *image_out, const UChar4Image *image_in) const;

		void FilterIntensity(FloatImage *image_out, const FloatImage *image_in) const;

		void FilterSubsample(UChar4Image *image_out, const UChar4Image *image_in) const;
		void FilterSubsample(FloatImage *image_out, const FloatImage *image_in) const;
		void FilterSubsampleWithHoles(FloatImage *image_out, const FloatImage *image_in) const;
		void FilterSubsampleWithHoles(Float4Image *image_out, const Float4Image *image_in) const;

		void GradientX(Short4Image *grad_out, const UChar4Image *image_in) const;
		void GradientY(Short4Image *grad_out, const UChar4Image *image_in) const;
		void GradientXY(Float2Image *grad_out, const FloatImage *image_in) const;

		int CountValidDepths(const FloatImage *image_in) const;

		LowLevelEngine_CUDA();
		~LowLevelEngine_CUDA();
	};
}
