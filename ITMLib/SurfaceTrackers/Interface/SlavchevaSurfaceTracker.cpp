//  ================================================================
//  Created by Gregory Kramida on 11/8/19.
//  Copyright (c)  2019 Gregory Kramida
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
#include <unordered_map>
#include <string>

//local
#include "SlavchevaSufraceTracker.h"
#include "../../Utils/Configuration.h"
#include "../../Utils/json_utils.h"

using namespace ITMLib;

//#pragma message BOOST_PP_STRINGIZE((DEFINE_INNER_SERIALIZABLE_STRUCT(SlavchevaSurfaceTracker, SWITCHES_STRUCT_DESCRIPTION)))

DEFINE_INNER_SERIALIZABLE_STRUCT(SlavchevaSurfaceTracker, PARAMETERS_STRUCT_DESCRIPTION)
DEFINE_INNER_SERIALIZABLE_STRUCT(SlavchevaSurfaceTracker, SWITCHES_STRUCT_DESCRIPTION)


SlavchevaSurfaceTracker::SlavchevaSurfaceTracker() :
		parameters(configuration::get().slavcheva_parameters),
		switches(configuration::get().slavcheva_switches) {
	PrintSettings();
}

SlavchevaSurfaceTracker::SlavchevaSurfaceTracker(SlavchevaSurfaceTracker::Switches switches,
                                                 SlavchevaSurfaceTracker::Parameters parameters) :
		parameters(parameters), switches(switches) {}

void SlavchevaSurfaceTracker::PrintSettings() {
	std::cout << bright_cyan << "*** Scene Motion Tracker Settings: ***" << reset << std::endl;
	std::cout << "Data term enabled: " << printBool(this->switches.enableDataTerm) << std::endl;
	std::cout << "Smoothing term enabled: " << printBool(this->switches.enableSmoothingTerm) << std::endl;
	std::cout << "Level Set term enabled: " << printBool(this->switches.enableLevelSetTerm) << std::endl;
	std::cout << "Killing term enabled: " << printBool(this->switches.enableKillingRigidityEnforcementTerm)
	          << std::endl;
	std::cout << "Gradient smoothing enabled: " << printBool(this->switches.enableSobolevGradientSmoothing) << std::endl
	          << std::endl;

	std::cout << "Gradient descent learning rate: " << this->parameters.gradientDescentLearningRate << std::endl;
	std::cout << "Rigidity enforcement factor: " << this->parameters.rigidityEnforcementFactor << std::endl;
	std::cout << "Weight of the data term: " << this->parameters.weightDataTerm << std::endl;
	std::cout << "Weight of the smoothness term: " << this->parameters.weightSmoothingTerm << std::endl;
	std::cout << "Weight of the level set term: " << this->parameters.weightLevelSetTerm << std::endl;
	std::cout << "Epsilon for the level set term: " << this->parameters.epsilon << std::endl;
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}