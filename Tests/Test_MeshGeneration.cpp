//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/27/20.
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
#define BOOST_TEST_MODULE MeshGeneration
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"

//ITMLib/ORUtils
#include "../ORUtils/MemoryDeviceType.h"
#include "../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Meshing/Mesh.h"
#include "../ITMLib/Engines/Meshing/MeshingEngineFactory.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericMeshSavingTest() {
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(configuration::get().device_type);
	Mesh mesh = meshing_engine->MeshScene( canonical_volume);
	mesh.WriteOBJ("TestData/meshes/mesh_partial_16.obj");


	delete canonical_volume;
	delete meshing_engine;
}

BOOST_AUTO_TEST_CASE(Test_MeshGeneration_CPU_VBH) {
	GenericMeshSavingTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

