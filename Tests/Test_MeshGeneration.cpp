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
	VoxelVolume<TSDFVoxel, TIndex>* volume;
	LoadVolume(&volume, snoopy::PartialVolume16Path<TIndex>(),
	           TMemoryDeviceType, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(TMemoryDeviceType);
	Mesh mesh = meshing_engine->MeshVolume( volume);
	ConstructGeneratedMeshDirectoryIfMissing();
	mesh.WriteOBJ(std::string(test_utilities::GeneratedMeshDirectory) + "mesh_partial_16_" + DeviceString<TMemoryDeviceType>() + ".obj");
	
	delete volume;
	delete meshing_engine;
}

#ifndef COMPILE_WITHOUT_CUDA
template<typename TIndex>
void GenericMeshGenerationAndComparisonTest_CUDA_to_CPU() {
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine_CPU =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU);
	MeshingEngine<TSDFVoxel, TIndex>* meshing_engine_CUDA =
			MeshingEngineFactory::Build<TSDFVoxel, TIndex>(MEMORYDEVICE_CUDA);

	//(CPU meshes)
	VoxelVolume<TSDFVoxel, TIndex>* volume_16_CPU;
	LoadVolume(&volume_16_CPU, snoopy::PartialVolume16Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	Mesh mesh_16_CPU = meshing_engine_CPU->MeshVolume(volume_16_CPU);

	VoxelVolume<TSDFVoxel, TIndex>* volume_17_CPU;
	LoadVolume(&volume_17_CPU, snoopy::PartialVolume17Path<TIndex>(),
	           MEMORYDEVICE_CPU, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	Mesh mesh_17_CPU = meshing_engine_CPU->MeshVolume(volume_17_CPU);

	//(CUDA meshes)
	VoxelVolume<TSDFVoxel, TIndex>* volume_16_CUDA;
	LoadVolume(&volume_16_CUDA, snoopy::PartialVolume16Path<TIndex>(),
	           MEMORYDEVICE_CUDA, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	Mesh mesh_16_CUDA = meshing_engine_CUDA->MeshVolume(volume_16_CUDA);

	VoxelVolume<TSDFVoxel, TIndex>* volume_17_CUDA;
	LoadVolume(&volume_17_CUDA, snoopy::PartialVolume17Path<TIndex>(),
	           MEMORYDEVICE_CUDA, snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	Mesh mesh_17_CUDA = meshing_engine_CUDA->MeshVolume(volume_17_CUDA);

	float absolute_tolerance = 1e-7;
	BOOST_REQUIRE(AlmostEqual(mesh_16_CPU, mesh_16_CUDA, absolute_tolerance));
	BOOST_REQUIRE(!AlmostEqual(mesh_17_CPU, mesh_16_CUDA, absolute_tolerance));
	BOOST_REQUIRE(AlmostEqual(mesh_17_CPU, mesh_17_CUDA, absolute_tolerance));

	delete volume_16_CPU;
	delete volume_17_CPU;
	delete volume_16_CUDA;
	delete volume_17_CUDA;
	delete meshing_engine_CPU;
	delete meshing_engine_CUDA;
}
#endif


BOOST_AUTO_TEST_CASE(Test_MeshGeneration_CPU_VBH) {
	GenericMeshSavingTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_MeshGeneration_CUDA_VBH) {
	GenericMeshSavingTest<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_MeshGeneration_CPU_to_CUDA){
	GenericMeshGenerationAndComparisonTest_CUDA_to_CPU<VoxelBlockHash>();
}

#endif
