//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/2/20.
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
#define BOOST_TEST_MODULE ImageProcessingEngine
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/TestDataUtilities.h"

//ORUtils
#include "../ORUtils/MemoryBlockPersistence/MemoryBlockPersistenceOperators.h"
#include "../ORUtils/FileUtils.h"

//ITMLib
#include "../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ITMLib/Utils/Analytics/RawArrayComparison.h"
#include "../ITMLib/Utils/Collections/OperationsOnSTLContainers.h"
#include "../ITMLib/Utils/Collections/MemoryBlock_StdContainer_Convertions.h"
#include "../ITMLib/Objects/Misc/PointCloud.h"

using namespace ITMLib;
using namespace test;

template<MemoryDeviceType TMemoryDeviceType>
void GenericConvertColorToIntensityTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	UChar4Image frame_115(teddy::frame_image_size, TMemoryDeviceType);
	ReadImageFromFile(frame_115, teddy::frame_115_color_file_name.Get());

	// perform the operation
	FloatImage frame_115_float_intensity(Vector2(0), TMemoryDeviceType);
	image_processing_engine->ConvertColorToIntensity(frame_115_float_intensity, frame_115);

	// load ground truth
	std::string float_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatIntensityImage.dat";
	ORUtils::IStreamWrapper file(float_intensity_path, true);
	FloatImage frame_115_float_intensity_gt(Vector2(0), TMemoryDeviceType);
	file >> frame_115_float_intensity_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_float_intensity == frame_115_float_intensity_gt);


	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterIntensityTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	FloatImage frame_115_float_intensity(teddy::frame_image_size, TMemoryDeviceType);
	std::string float_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatIntensityImage.dat";
	ORUtils::IStreamWrapper input_file(float_intensity_path, true);
	input_file >> frame_115_float_intensity;

	// perform the operation
	FloatImage frame_115_filtered_intensity(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterIntensity(frame_115_filtered_intensity, frame_115_float_intensity);

	// load ground truth
	std::string filtered_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FilteredIntensityImage.dat";
	ORUtils::IStreamWrapper ground_truth_file(filtered_intensity_path, true);
	FloatImage frame_115_filtered_intensity_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_filtered_intensity_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_filtered_intensity == frame_115_filtered_intensity_gt);

	delete image_processing_engine;
}



template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterSubsampleUchar4Test(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	UChar4Image frame_115(teddy::frame_image_size, TMemoryDeviceType);
	ReadImageFromFile(frame_115, teddy::frame_115_color_file_name.Get());

	// perform the operation
	UChar4Image frame_115_subsampled(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsample(frame_115_subsampled, frame_115);

	// load ground truth
	std::string subsampled_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_SubsampledImage.dat";
	ORUtils::IStreamWrapper ground_truth_file(subsampled_path, true);
	UChar4Image frame_115_subsampled_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_subsampled_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_subsampled == frame_115_subsampled_gt);

	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterSubsampleFloatTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	FloatImage frame_115_float_intensity(teddy::frame_image_size, TMemoryDeviceType);
	std::string float_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatIntensityImage.dat";
	ORUtils::IStreamWrapper input_file(float_intensity_path, true);
	input_file >> frame_115_float_intensity;

	// perform the operation
	FloatImage frame_115_float_subsampled(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsample(frame_115_float_subsampled, frame_115_float_intensity);

	// load ground truth
	std::string float_subsampled_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatSubsampledImage.dat";
	ORUtils::IStreamWrapper ground_truth_file(float_subsampled_path, true);
	FloatImage frame_115_float_subsampled_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_float_subsampled_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_float_subsampled == frame_115_float_subsampled_gt);

	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterSubsampleWithHolesFloatTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	FloatImage depth(teddy::frame_image_size, TMemoryDeviceType);
	std::string depth_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_Depth.dat";
	ORUtils::IStreamWrapper input_file(depth_path, true);
	input_file >> depth;

	// perform the operation
	FloatImage frame_115_depth_subsampled_with_holes(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsampleWithHoles(frame_115_depth_subsampled_with_holes, depth);

	// load ground truth
	std::string depth_subsampled_with_holes_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_DepthSubsampledWithHolesImage.dat";
	ORUtils::IStreamWrapper ground_truth_file(depth_subsampled_with_holes_path, true);
	FloatImage frame_115_depth_subsampled_with_holes_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_depth_subsampled_with_holes_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_depth_subsampled_with_holes == frame_115_depth_subsampled_with_holes_gt);

	delete image_processing_engine;
}


template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterSubsampleWithHolesFloat4Test(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original point cloud
	PointCloud point_cloud(teddy::frame_image_size,TMemoryDeviceType);
	std::string icp_point_cloud_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_PointCloud.dat";
	ORUtils::IStreamWrapper input_file(icp_point_cloud_path, true);
	input_file >> point_cloud;

	// perform the operation
	Float4Image frame_115_points_subsampled_with_holes(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsampleWithHoles(frame_115_points_subsampled_with_holes, point_cloud.locations);

	// load ground truth
	std::string subsampled_points_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_SubsampledPoints.dat";
	ORUtils::IStreamWrapper ground_truth_file(subsampled_points_path, true);
	Float4Image frame_115_points_subsampled_with_holes_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_points_subsampled_with_holes_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_points_subsampled_with_holes == frame_115_points_subsampled_with_holes_gt);

	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientXTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	
	// load original image
	UChar4Image frame_115(teddy::frame_image_size, TMemoryDeviceType);
	ReadImageFromFile(frame_115, teddy::frame_115_color_file_name.Get());

	// perform the operation
	Short4Image frame_115_gradient_x(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientX(frame_115_gradient_x, frame_115);

	// load ground truth
	std::string gradient_x_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientX.dat";
	ORUtils::IStreamWrapper ground_truth_file(gradient_x_path, true);
	Short4Image frame_115_gradient_x_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_gradient_x_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_gradient_x == frame_115_gradient_x_gt);

	delete image_processing_engine;
}


template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientYTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	UChar4Image frame_115(teddy::frame_image_size, TMemoryDeviceType);
	ReadImageFromFile(frame_115, teddy::frame_115_color_file_name.Get());

	// perform the operation
	Short4Image frame_115_gradient_y(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientY(frame_115_gradient_y, frame_115);

	// load ground truth
	std::string gradient_y_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientY.dat";
	ORUtils::IStreamWrapper ground_truth_file(gradient_y_path, true);
	Short4Image frame_115_gradient_y_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_gradient_y_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_gradient_y == frame_115_gradient_y_gt);

	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientXYTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	// load original image
	FloatImage frame_115_float_intensity(teddy::frame_image_size, TMemoryDeviceType);
	std::string float_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatIntensityImage.dat";
	ORUtils::IStreamWrapper input_file(float_intensity_path, true);
	input_file >> frame_115_float_intensity;

	// perform the operation
	Float2Image frame_115_gradient_xy(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientXY(frame_115_gradient_xy, frame_115_float_intensity);

	// load ground truth
	std::string gradient_xy_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientXY.dat";
	ORUtils::IStreamWrapper ground_truth_file(gradient_xy_path, true);
	Float2Image frame_115_gradient_xy_gt(Vector2(0), TMemoryDeviceType);
	ground_truth_file >> frame_115_gradient_xy_gt;

	// compare output and ground truth
	BOOST_REQUIRE(frame_115_gradient_xy == frame_115_gradient_xy_gt);

	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void CountValidDepthsTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	FloatImage depth(teddy::frame_image_size, TMemoryDeviceType);
	std::string depth_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_Depth.dat";
	ORUtils::IStreamWrapper input_file(depth_path, true);
	input_file >> depth;

	int valid_depths = image_processing_engine->CountValidDepths(depth);
	std::string valid_depth_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_ValidDepthCount.txt";
	int valid_depths_gt;
	std::ifstream file;
	file.open(valid_depth_path);
	file >> valid_depths_gt;
	file.close();

	BOOST_REQUIRE_EQUAL(valid_depths, valid_depths_gt);
	delete image_processing_engine;
}


BOOST_AUTO_TEST_CASE(Test_ConvertColorToIntensity_CPU) {
	GenericConvertColorToIntensityTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FilterIntensity_CPU) {
	GenericFilterIntensityTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FilterSubsampleUchar4_CPU) {
	GenericFilterSubsampleUchar4Test<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FilterSubsampleFloat_CPU) {
	GenericFilterSubsampleFloatTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FilterSubsampleWithHolesFloat_CPU) {
	GenericFilterSubsampleWithHolesFloatTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FilterSubsampleWithHolesFloat4_CPU) {
	GenericFilterSubsampleWithHolesFloat4Test<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_GradientX_CPU) {
	GenericGradientXTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_GradientY_CPU) {
	GenericGradientYTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_GradientXY_CPU) {
	GenericGradientXYTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CountValidDepths_CPU) {
	CountValidDepthsTest<MEMORYDEVICE_CPU>();
}