//  ================================================================
//  Created by Gregory Kramida on 9/3/19.
//  Copyright (c) 2019 Gregory Kramida
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
#include <iostream>

#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <iostream>

//boost
#include <boost/test/unit_test.hpp>

//test targets
#include "../ORUtils/FileUtils.h"
#include "../InputSource/ImageSourceEngine.h"

//test_utils
#include "TestUtilities/SnoopyTestUtilities.h"

namespace snoopy = snoopy_test_utilities;

//#define GENERATE_GT_MASKED_IMAGES

BOOST_AUTO_TEST_CASE(testImageMaskReader) {

	using namespace InputSource;
	UChar4Image rgb(true, false);
	ShortImage depth(true, false);
	UChar4Image masked_rgb_ground_truth(true, false);
	ShortImage masked_depth_ground_truth(true, false);
	UCharImage mask(true, false);

	InputSource::ImageMaskPathGenerator pathGenerator(
			GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_color_%06i.png",
			GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_%06i.png",
			GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_omask_%06i.png");
	InputSource::ImageSourceEngine* image_source =
			new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>(
			snoopy::SnoopyCalibrationPath().c_str(), pathGenerator);
	image_source->GetImages(rgb, depth);

#ifdef GENERATE_GT_MASKED_IMAGES
	BOOST_REQUIRE(ReadImageFromFile(rgb, GENERATED_TEST_DATA_PREFIX "TestData/snoopy_color_000000.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, GENERATED_TEST_DATA_PREFIX "TestData/snoopy_depth_000000.png"));
#endif
	BOOST_REQUIRE(ReadImageFromFile(mask, GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_omask_000000.png"));

	rgb.ApplyMask(mask,Vector4u((unsigned char)0));
	depth.ApplyMask(mask,0);

#ifdef GENERATE_GT_MASKED_IMAGES
	SaveImageToFile(rgb, GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_color_000000_masked.pnm");
	SaveImageToFile(depth, GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_000000_masked.pnm");
#endif

BOOST_REQUIRE(ReadImageFromFile(masked_rgb_ground_truth, GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_color_000000.png"));
	masked_rgb_ground_truth.ApplyMask(mask, Vector4u((unsigned char) 0));
	BOOST_REQUIRE(ReadImageFromFile(masked_depth_ground_truth, GENERATED_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_000000_masked.pnm"));

	BOOST_REQUIRE(rgb == masked_rgb_ground_truth);
	BOOST_REQUIRE(depth == masked_depth_ground_truth);

}