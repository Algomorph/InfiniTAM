//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 8/24/20.
//  Copyright (c) 2020 Gregory Kramida
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
#define BOOST_TEST_MODULE FFmpegReadWrite
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"

//ORUtils
#include "../../ORUtils/FileUtils.h"

//InputSource
#include "../../InputSource/FFMPEGReader.h"
#include "../../InputSource/FFMPEGWriter.h"

using namespace InputSource;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

BOOST_AUTO_TEST_CASE(TestFFmpegRead) {
	FFMPEGReader reader(snoopy::SnoopyCalibrationPath().c_str(), snoopy::VideoFrames16to18ColorPath().c_str(),
	                    snoopy::VideoFrames16to18DepthPath().c_str());
	UChar4Image color(reader.GetRGBImageSize(), MEMORYDEVICE_CPU);
	ShortImage depth(reader.GetDepthImageSize(), MEMORYDEVICE_CPU);

	// *** Frame 16 ***
	BOOST_REQUIRE(reader.HasMoreImages() == true);
	reader.GetImages(color, depth);

	UChar4Image frame_16_color(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_16_color, snoopy::Frame16ColorPath().c_str());
	ShortImage frame_16_depth(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_16_depth, snoopy::Frame16DepthPath().c_str());

	BOOST_REQUIRE(depth == frame_16_depth);
	BOOST_REQUIRE(color == frame_16_color);

	// *** Frame 17 ***
	BOOST_REQUIRE(reader.HasMoreImages() == true);
	reader.GetImages(color, depth);

	UChar4Image frame_17_color(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_17_color, snoopy::Frame17ColorPath().c_str());
	ShortImage frame_17_depth(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_17_depth, snoopy::Frame17DepthPath().c_str());

	BOOST_REQUIRE(depth == frame_17_depth);
	BOOST_REQUIRE(color == frame_17_color);

	// *** Frame 18 ***
	BOOST_REQUIRE(reader.HasMoreImages() == true);
	reader.GetImages(color, depth);


	UChar4Image frame_18_color(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_18_color, snoopy::Frame18ColorPath().c_str());
	ShortImage frame_18_depth(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ReadImageFromFile(frame_18_depth, snoopy::Frame18DepthPath().c_str());

	BOOST_REQUIRE(depth == frame_18_depth);
	BOOST_REQUIRE(color == frame_18_color);

	BOOST_REQUIRE(reader.HasMoreImages() == false);

}

BOOST_AUTO_TEST_CASE(TestFFmpegWrite) {
	FFMPEGWriter writer_color;
	FFMPEGWriter writer_depth;
	test_utilities::ConstructGeneratedVideosDirectoryIfMissing();
	writer_color.open((std::string(test_utilities::GeneratedVideosDirectory) + "snoopy_color_16-18_test.avi").c_str(),
	                  snoopy::frame_image_size.width, snoopy::frame_image_size.height, false, 3);
	writer_depth.open((std::string(test_utilities::GeneratedVideosDirectory) + "snoopy_depth_16-18_test.avi").c_str(),
	                  snoopy::frame_image_size.width, snoopy::frame_image_size.height, true, 3);

	UChar4Image color(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ShortImage depth(snoopy::frame_image_size, MEMORYDEVICE_CPU);

	// *** write all frames ***
	ReadImageFromFile(color, snoopy::Frame16ColorPath().c_str());
	ReadImageFromFile(depth, snoopy::Frame16DepthPath().c_str());
	writer_color.writeFrame(&color);
	writer_depth.writeFrame(&depth);

	ReadImageFromFile(color, snoopy::Frame17ColorPath().c_str());
	ReadImageFromFile(depth, snoopy::Frame17DepthPath().c_str());
	writer_color.writeFrame(&color);
	writer_depth.writeFrame(&depth);

	ReadImageFromFile(color, snoopy::Frame18ColorPath().c_str());
	ReadImageFromFile(depth, snoopy::Frame18DepthPath().c_str());
	writer_color.writeFrame(&color);
	writer_depth.writeFrame(&depth);

	writer_color.close();
	writer_depth.close();

	FFMPEGReader reader(snoopy::SnoopyCalibrationPath().c_str(),
	                    (std::string(test_utilities::GeneratedVideosDirectory) + "snoopy_color_16-18_test.avi").c_str(),
	                    (std::string(test_utilities::GeneratedVideosDirectory) + "snoopy_depth_16-18_test.avi").c_str());
	FFMPEGReader reader_gt(snoopy::SnoopyCalibrationPath().c_str(),
	                       snoopy::VideoFrames16to18Color_YUV422P_Path().c_str(),
	                       snoopy::VideoFrames16to18Depth_GRAY16LE_Path().c_str());
	UChar4Image color_gt(snoopy::frame_image_size, MEMORYDEVICE_CPU);
	ShortImage depth_gt(snoopy::frame_image_size, MEMORYDEVICE_CPU);

	// *** Test Frame 16 ***
	reader.GetImages(color, depth);
	reader_gt.GetImages(color_gt, depth_gt);

	BOOST_REQUIRE(color == color_gt);
	BOOST_REQUIRE(depth == depth_gt);

	// *** Test Frame 17 ***
	reader.GetImages(color, depth);
	reader_gt.GetImages(color_gt, depth_gt);

	BOOST_REQUIRE(color == color_gt);
	BOOST_REQUIRE(depth == depth_gt);

	// *** Test Frame 18 ***
	reader.GetImages(color, depth);
	reader_gt.GetImages(color_gt, depth_gt);

	BOOST_REQUIRE(color == color_gt);
	BOOST_REQUIRE(depth == depth_gt);


	BOOST_REQUIRE(reader.HasMoreImages() == false);
}