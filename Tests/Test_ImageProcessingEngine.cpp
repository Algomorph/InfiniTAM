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
#include "../ORUtils/IStreamWrapper.h"

//ITMLib
#include "../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ITMLib/Utils/Analytics/RawArrayComparison.h"
#include "../ITMLib/Utils/Collections/OperationsOnSTLContainers.h"
#include "../ITMLib/Utils/Collections/MemoryBlock_StdContainer_Convertions.h"

using namespace ITMLib;
using namespace test;

template<MemoryDeviceType TMemoryDeviceType>
void GenericFilterSubsampleTest() {

}

BOOST_AUTO_TEST_CASE(Test_GenericFilterSubsampleTest_CPU) {
	GenericFilterSubsampleTest<MEMORYDEVICE_CPU>();
}