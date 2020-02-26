//  ================================================================
//  Created by Gregory Kramida on 12/12/19.
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
#define BOOST_TEST_MODULE CUDA_Atomics
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <iostream>

//boost
#include <boost/test/unit_test.hpp>
#include <iostream>

//local
#include "CUDAAtomicTesting.h"


BOOST_AUTO_TEST_CASE(TestAtomicAddChar){
	std::cout << "*** Atomic add test on char ***" << std::endl << std::endl;
	BOOST_REQUIRE(AtomicAddCharTest());
}

BOOST_AUTO_TEST_CASE(TestAtomicAddShort){
	std::cout << "*** Atomic add test on short ***" << std::endl << std::endl;
	BOOST_REQUIRE(AtomicAddShortTest());
}

BOOST_AUTO_TEST_CASE(TestAtomicCASChar){
	std::cout << "*** Atomic CAS test on char ***" << std::endl << std::endl;
	BOOST_REQUIRE(AtomicCASCharTest());
}
