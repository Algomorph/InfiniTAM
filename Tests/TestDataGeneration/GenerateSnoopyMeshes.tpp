//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/24/20.
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
// === local ===
#include "GenerateSnoopyMeshes.h"
// === ITMLib ===
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Engines/Meshing/MeshingEngineFactory.h"
// === Test Utilities ===
#include "../TestUtilities/TestDataUtilities.h"

using namespace ITMLib;
using namespace test;

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateMeshingTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating meshing test data (" << IndexString<TIndex>() << ") ...");
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, test::snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);
	Mesh mesh = meshing_engine->MeshVolume(canonical_volume);
	ConstructGeneratedMeshDirectoryIfMissing();
	mesh.WriteOBJ(GENERATED_TEST_DATA_PREFIX "TestData/meshes/mesh_partial_16.obj");

	delete canonical_volume;
	delete meshing_engine;
}