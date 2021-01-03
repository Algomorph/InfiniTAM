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

template<MemoryDeviceType TMemoryDeviceType, typename TImageType, bool TWithHoles>
void GenericFilterSubsampleTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientXTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientYTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericGradientXYTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	delete image_processing_engine;
}

template<MemoryDeviceType TMemoryDeviceType>
void CountValidDepthsTest(){
	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	delete image_processing_engine;
}


BOOST_AUTO_TEST_CASE(Test_ConvertColorToIntensityTest_CPU) {
	GenericConvertColorToIntensityTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_ConvertColorToIntensityTest_CPU) {
	GenericConvertColorToIntensityTest<MEMORYDEVICE_CPU>();
}