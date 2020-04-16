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

//#define GENERATE_GT_MASKED_IMAGES

BOOST_AUTO_TEST_CASE(testImageMaskReader) {

	using namespace InputSource;
	ITMUChar4Image rgb(true, false);
	ITMShortImage depth(true, false);
	ITMUChar4Image gtMaskedRgb(true, false);
	ITMShortImage gtMaskedDepth(true, false);
	ITMUCharImage mask(true, false);

	InputSource::ImageMaskPathGenerator pathGenerator("TestData/snoopy_color_%06i.png", "TestData/snoopy_depth_%06i.png",
	                                                  "TestData/snoopy_omask_%06i.png");
	InputSource::ImageSourceEngine* imageSource = new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>(
			"TestData/snoopy_calib.txt", pathGenerator);
	imageSource->GetImages(rgb, depth);

#ifdef GENERATE_GT_MASKED_IMAGES
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/snoopy_color_000000.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/snoopy_depth_000000.png"));
#endif
	BOOST_REQUIRE(ReadImageFromFile(mask, "TestData/snoopy_omask_000000.png"));

	rgb.ApplyMask(mask,Vector4u((unsigned char)0));
	depth.ApplyMask(mask,0);

#ifdef GENERATE_GT_MASKED_IMAGES
	SaveImageToFile(rgb, "TestData/snoopy_color_000000_masked.pnm");
	SaveImageToFile(depth, "TestData/snoopy_depth_000000_masked.pnm");
#endif

BOOST_REQUIRE(ReadImageFromFile(gtMaskedRgb, "TestData/snoopy_color_000000.png"));
	gtMaskedRgb.ApplyMask(mask, Vector4u((unsigned char) 0));
	BOOST_REQUIRE(ReadImageFromFile(gtMaskedDepth, "TestData/snoopy_depth_000000_masked.pnm"));

	BOOST_REQUIRE(rgb == gtMaskedRgb);
	BOOST_REQUIRE(depth == gtMaskedDepth);

}