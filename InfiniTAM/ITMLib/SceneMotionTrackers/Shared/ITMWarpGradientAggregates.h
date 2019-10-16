//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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
#pragma once
#include "../../../ORUtils/PlatformIndependentAtomics.h"

struct AdditionalGradientAggregates{
	AdditionalGradientAggregates(){
		INITIALIZE_ATOMIC(cumulativeCanonicalSdf, 0.f);
		INITIALIZE_ATOMIC(cumulativeLiveSdf,0.f);
		INITIALIZE_ATOMIC(cumulativeSdfDiff, 0.f);
		INITIALIZE_ATOMIC(cumulativeWarpDist, 0.f);

		INITIALIZE_ATOMIC(consideredVoxelCount, 0u);
		INITIALIZE_ATOMIC(dataVoxelCount, 0u);
		INITIALIZE_ATOMIC(levelSetVoxelCount, 0u);
	}
	~AdditionalGradientAggregates(){
		CLEAN_UP_ATOMIC(cumulativeCanonicalSdf);
		CLEAN_UP_ATOMIC(cumulativeLiveSdf);
		CLEAN_UP_ATOMIC(cumulativeSdfDiff);
		CLEAN_UP_ATOMIC(cumulativeWarpDist);

		CLEAN_UP_ATOMIC(consideredVoxelCount);
		CLEAN_UP_ATOMIC(dataVoxelCount);
		CLEAN_UP_ATOMIC(levelSetVoxelCount);
	}

	DECLARE_ATOMIC_FLOAT(cumulativeCanonicalSdf);
	DECLARE_ATOMIC_FLOAT(cumulativeLiveSdf);
	DECLARE_ATOMIC_FLOAT(cumulativeSdfDiff);
	DECLARE_ATOMIC_FLOAT(cumulativeWarpDist);

	DECLARE_ATOMIC_UINT(consideredVoxelCount);
	DECLARE_ATOMIC_UINT(dataVoxelCount);
	DECLARE_ATOMIC_UINT(levelSetVoxelCount);
};

struct ComponentEnergies{
	ComponentEnergies(){
		INITIALIZE_ATOMIC(totalDataEnergy, 0.f);
		INITIALIZE_ATOMIC(totalLevelSetEnergy,0.f);
		INITIALIZE_ATOMIC(totalTikhonovEnergy,0.f);
		INITIALIZE_ATOMIC(totalRigidityEnergy,0.f);
	}
	~ComponentEnergies(){
		CLEAN_UP_ATOMIC(totalDataEnergy);
		CLEAN_UP_ATOMIC(totalLevelSetEnergy);
		CLEAN_UP_ATOMIC(totalTikhonovEnergy);
		CLEAN_UP_ATOMIC(totalRigidityEnergy);
	}
	DECLARE_ATOMIC_FLOAT(totalDataEnergy);
	DECLARE_ATOMIC_FLOAT(totalLevelSetEnergy);
	DECLARE_ATOMIC_FLOAT(totalTikhonovEnergy);
	DECLARE_ATOMIC_FLOAT(totalRigidityEnergy);
};

