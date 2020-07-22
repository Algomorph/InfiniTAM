//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/8/20.
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
#pragma once

#include "../../Utils/Configuration/Configuration.h"
#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"

namespace ITMLib {
template<typename TParameters>
class Configurable {
protected:
	const TParameters parameters;
public: // instance functions
	Configurable() :
			parameters(
					ExtractDeferrableSerializableStructFromPtreeIfPresent<TParameters>(
							configuration::get().source_tree,
							configuration::get().origin
					)
			) {}

	Configurable(configuration::Configuration& config) :
			parameters(
					ExtractDeferrableSerializableStructFromPtreeIfPresent<TParameters>(
							config.source_tree,
							config.origin
					)
			) {}

	Configurable(TParameters) : parameters(parameters) {};

	const TParameters GetParameters() const { return this->parameters; }
};

} // namespace ITMLib

