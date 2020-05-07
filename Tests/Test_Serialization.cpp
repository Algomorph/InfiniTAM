//  ================================================================
//  Created by Gregory Kramida on 1/6/20.
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
#define BOOST_TEST_MODULE Serialization
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <iostream>

//boost
#include <boost/test/unit_test.hpp>

#include "../ITMLib/Utils/Metacoding/Metacoding.h"

#define VERBOSITY2_LEVEL_ENUM_DESCRIPTION Verbosity2Level, \
	(VERBOSITY2_SILENT, "silent", "SILENT", "VERBOSITY_WARNING"), \
	(VERBOSITY2_TOP_LEVEL, "top_level", "TOP_LEVEL", "Top-level operations", "VERBOSITY_TOP_LEVEL", "top-level", "top-level operations"), \
	(VERBOSITY2_PER_FRAME, "per_frame", "PER_FRAME", "Per-frame operations", "VERBOSITY_PER_FRAME", "per-frame", "per-frame operations"), \
	(VERBOSITY2_PER_ITERATION, "per_iteration", "PER_ITERATION", "Per-iteration operations", "VERBOSITY_PER_ITERATION", "per-iteration", "per-iteration operations")

class OuterClass{
public:
	DECLARE_SERIALIZABLE_ENUM(VERBOSITY2_LEVEL_ENUM_DESCRIPTION )
};

DEFINE_SERIALIZABLE_ENUM(OuterClass::VERBOSITY2_LEVEL_ENUM_DESCRIPTION);

BOOST_AUTO_TEST_CASE(TestSerializableEnum){
	OuterClass::Verbosity2Level v2l01, v2l02, v2l03, v2l04;
	v2l01 = string_to_enumerator<OuterClass::Verbosity2Level>("silent");
	v2l02 = string_to_enumerator<OuterClass::Verbosity2Level>("SILENT");
	v2l03 = string_to_enumerator<OuterClass::Verbosity2Level>("VERBOSITY_WARNING");

	BOOST_REQUIRE_EQUAL(v2l01, OuterClass::VERBOSITY2_SILENT);
	BOOST_REQUIRE_EQUAL(v2l02, OuterClass::VERBOSITY2_SILENT);
	BOOST_REQUIRE_EQUAL(v2l03, OuterClass::VERBOSITY2_SILENT);

	v2l01 = string_to_enumerator<OuterClass::Verbosity2Level>("top_level");
	v2l02 = string_to_enumerator<OuterClass::Verbosity2Level>("VERBOSITY_TOP_LEVEL");
	v2l03 = string_to_enumerator<OuterClass::Verbosity2Level>("top-level");
	v2l04 = string_to_enumerator<OuterClass::Verbosity2Level>("top-level operations");

	BOOST_REQUIRE_EQUAL(v2l01, OuterClass::VERBOSITY2_TOP_LEVEL);
	BOOST_REQUIRE_EQUAL(v2l02, OuterClass::VERBOSITY2_TOP_LEVEL);
	BOOST_REQUIRE_EQUAL(v2l03, OuterClass::VERBOSITY2_TOP_LEVEL);
	BOOST_REQUIRE_EQUAL(v2l04, OuterClass::VERBOSITY2_TOP_LEVEL);

	v2l01 = string_to_enumerator<OuterClass::Verbosity2Level>("per_frame");
	v2l02 = string_to_enumerator<OuterClass::Verbosity2Level>("PER_FRAME");
	v2l03 = string_to_enumerator<OuterClass::Verbosity2Level>("Per-frame operations");
	v2l04 = string_to_enumerator<OuterClass::Verbosity2Level>("per-frame operations");

	BOOST_REQUIRE_EQUAL(v2l01, OuterClass::VERBOSITY2_PER_FRAME);
	BOOST_REQUIRE_EQUAL(v2l02, OuterClass::VERBOSITY2_PER_FRAME);
	BOOST_REQUIRE_EQUAL(v2l03, OuterClass::VERBOSITY2_PER_FRAME);
	BOOST_REQUIRE_EQUAL(v2l04, OuterClass::VERBOSITY2_PER_FRAME);

	v2l01 = string_to_enumerator<OuterClass::Verbosity2Level>("per_iteration");
	v2l02 = string_to_enumerator<OuterClass::Verbosity2Level>("PER_ITERATION");
	v2l03 = string_to_enumerator<OuterClass::Verbosity2Level>("per-iteration");
	v2l04 = string_to_enumerator<OuterClass::Verbosity2Level>("per-iteration operations");

	BOOST_REQUIRE_EQUAL(v2l01, OuterClass::VERBOSITY2_PER_ITERATION);
	BOOST_REQUIRE_EQUAL(v2l02, OuterClass::VERBOSITY2_PER_ITERATION);
	BOOST_REQUIRE_EQUAL(v2l03, OuterClass::VERBOSITY2_PER_ITERATION);
	BOOST_REQUIRE_EQUAL(v2l04, OuterClass::VERBOSITY2_PER_ITERATION);

	try{
		OuterClass::Verbosity2Level v2l01 = string_to_enumerator<OuterClass::Verbosity2Level>("blah");
		BOOST_TEST_FAIL("Expression should have thrown a runtime_error, and it hasn't.");
	} catch (std::runtime_error& e) {
		std::cout << "Error caught; test succeeded." << std::endl;
	}

	std::string token1, token2, token3, token4;
	token1 = enumerator_to_string(OuterClass::VERBOSITY2_SILENT);
	token2 = enumerator_to_string(OuterClass::VERBOSITY2_TOP_LEVEL);
	token3 = enumerator_to_string(OuterClass::VERBOSITY2_PER_FRAME);
	token4 = enumerator_to_string(OuterClass::VERBOSITY2_PER_ITERATION);

	BOOST_REQUIRE_EQUAL(token1, "silent");
	BOOST_REQUIRE_EQUAL(token2, "top_level");
	BOOST_REQUIRE_EQUAL(token3, "per_frame");
	BOOST_REQUIRE_EQUAL(token4, "per_iteration");
}