#pragma once

#include "../../../Utils/ImageTypes.h"

namespace ITMLib
{
	/// Interface to low level image processing engines.
	class PreprocessingEngineInterface
	{
	public:
		virtual void CopyImage(UChar4Image *image_out, const UChar4Image *image_in) const = 0;
		virtual void CopyImage(FloatImage *image_out, const FloatImage *image_in) const = 0;
		virtual void CopyImage(Float4Image *image_out, const Float4Image *image_in) const = 0;

		virtual void ConvertColourToIntensity(FloatImage *image_out, const UChar4Image *image_in) const = 0;

		virtual void FilterIntensity(FloatImage *image_out, const FloatImage *image_in) const = 0;

		virtual void FilterSubsample(UChar4Image *image_out, const UChar4Image *image_in) const = 0;
		virtual void FilterSubsample(FloatImage *image_out, const FloatImage *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(FloatImage *image_out, const FloatImage *image_in) const = 0;
		virtual void FilterSubsampleWithHoles(Float4Image *image_out, const Float4Image *image_in) const = 0;

		virtual void GradientX(Short4Image *grad_out, const UChar4Image *image_in) const = 0;
		virtual void GradientY(Short4Image *grad_out, const UChar4Image *image_in) const = 0;
		virtual void GradientXY(Float2Image *grad_out, const FloatImage *image_in) const = 0;

		virtual int CountValidDepths(const FloatImage *image_in) const = 0;

		PreprocessingEngineInterface() { }
		virtual ~PreprocessingEngineInterface() { }
	};
}
