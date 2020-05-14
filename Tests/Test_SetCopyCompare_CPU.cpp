//  ================================================================
//  Created by Gregory Kramida on 10/23/17.
//  Copyright (c) 2017-2000 Gregory Kramida
//  Licensed under the Apache Li
// cense, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#define BOOST_TEST_MODULE SetCopyCompare_CPU
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <random>
#include <vector>


//boost
#include <boost/test/unit_test.hpp>

//ITMlib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Objects/Volume/RepresentationAccess.h"
#include "../ITMLib/Objects/Camera/CalibIO.h"
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../InputSource/ImageSourceEngine.h"
#include "../ORUtils/FileUtils.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"

using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_PlainVoxelArray_CPU) {
	Vector3i volumeSize(20);
	Vector3i volumeOffset(-10, -10, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> scene1(MEMORYDEVICE_CPU,
	                                               PlainVoxelArray::InitializationParameters(volumeSize, volumeOffset, ""));


	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&scene1);

	TSDFVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	TSDFVoxel out;
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	TSDFVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(3, 3, 3), voxelHalf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(8, 5, 2);
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene1, voxelPos, voxelZero);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-2, 3, 4);
	VoxelVolume<TSDFVoxel, PlainVoxelArray> scene2(MEMORYDEVICE_CPU, {volumeSize, volumeOffset});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&scene2);

	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolume(&scene2, &scene1, offset);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(3, 3, 3) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_VoxelBlockHash_CPU) {
	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene1(MEMORYDEVICE_CPU, {0x800, 0x20000});

	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene1);

	TSDFVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	TSDFVoxel out;
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	TSDFVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232, 125, 62);
	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene1, voxelPos, voxelZero);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-34, 6, 9);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene2(MEMORYDEVICE_CPU, {0x800, 0x20000});

	ManipulationEngine_CPU_VBH_Voxel::Inst().CopyVolume(&scene2, &scene1, offset);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(9, 9, 9) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testCopyToDifferentlyInitializedVolume_VBH_CPU){
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CPU,
	                                              snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume.Reset();
	std::string path = snoopy::PartialVolume17Path<VoxelBlockHash>();

	volume.LoadFromDisk(path);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_copy(MEMORYDEVICE_CPU,
	                                                   {snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>().voxel_block_count * 2,
	                                                    snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>().excess_list_size});
	volume_copy.Reset();
	ManipulationEngine_CPU_VBH_Voxel::Inst().CopyVolume(&volume_copy, &volume);
	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_copy, &volume, tolerance));
}

BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CPU_ITMVoxel) {
	float tolerance = 1e-6;

	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA1(MEMORYDEVICE_CPU,
	                                                    {volumeSize, volumeOffset});
	volume_PVA1.Reset();
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA2(MEMORYDEVICE_CPU,
	                                                    {volumeSize, volumeOffset});
	volume_PVA2.Reset();
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH1(MEMORYDEVICE_CPU,
	                                                   {0x800, 0x20000});
	volume_VBH1.Reset();
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH2(MEMORYDEVICE_CPU,
	                                                   {0x800, 0x20000});
	volume_VBH2.Reset();

	std::random_device random_device;
	std::mt19937 generator(random_device());
	int single_voxel_tests_run_counter = 0;

	auto single_voxel_tests = [&]() {
		std::cout << "Single voxel test run " << single_voxel_tests_run_counter << std::endl;
		single_voxel_tests_run_counter++;
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		TSDFVoxel voxel;
		SimulateVoxelAlteration(voxel, -0.1f);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA2, coordinate, voxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH2, coordinate, voxel);

		std::cout << "Altering single voxel at: " << coordinate << std::endl;

		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_PVA1, &volume_PVA2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_VBH1, &volume_VBH2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_PVA1, &volume_VBH2, tolerance));

		TSDFVoxel defaultVoxel;
		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA2, coordinate, defaultVoxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH2, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_PVA1, &volume_PVA2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_VBH1, &volume_VBH2, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		voxel = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&volume_PVA2, coordinate);
		SimulateVoxelAlteration(voxel, fmod((TSDFVoxel::valueToFloat(voxel.sdf) + 0.1f), 1.0f));
		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA2, coordinate, voxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH2, coordinate, voxel);

		std::cout << "Altering single voxel at: " << coordinate << std::endl;

		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_PVA1, &volume_PVA2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_VBH1, &volume_VBH2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&volume_PVA1, &volume_VBH2, tolerance));

		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA2, coordinate, defaultVoxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH2, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> sdf_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x - 1);

   const int modified_voxel_count = 120;

	single_voxel_tests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iVoxel = 0; iVoxel < modified_voxel_count; iVoxel++) {
		TSDFVoxel voxel;
		SimulateVoxelAlteration(voxel, sdf_distribution(generator));
		voxel.print_self();
		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA1, coordinate, voxel);
		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&volume_PVA2, coordinate, voxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH1, coordinate, voxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&volume_VBH2, coordinate, voxel);

	}

	BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_PVA1, &volume_PVA2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&volume_VBH1, &volume_VBH2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_PVA1, &volume_VBH1, tolerance, OPTIMIZED));

	single_voxel_tests();
}

BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CPU_ITMWarp) {
	float tolerance = 1e-6;
	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;

	VoxelVolume<WarpVoxel, PlainVoxelArray> scene1(MEMORYDEVICE_CPU,
	                                               {volumeSize, volumeOffset});
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetVolume(&scene1);
	VoxelVolume<WarpVoxel, PlainVoxelArray> scene2(MEMORYDEVICE_CPU,
	                                               {volumeSize, volumeOffset});
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetVolume(&scene2);
	VoxelVolume<WarpVoxel, VoxelBlockHash> scene3(MEMORYDEVICE_CPU,
	                                              {0x800, 0x20000});
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetVolume(&scene3);
	VoxelVolume<WarpVoxel, VoxelBlockHash> scene4(MEMORYDEVICE_CPU,
	                                              {0x800, 0x20000});
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetVolume(&scene4);

	std::random_device random_device;
	std::mt19937 generator(random_device());

	auto singleVoxelTests = [&]() {
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		WarpVoxel warp;
		warp.warp_update = Vector3f(-0.1);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		WarpVoxel defaultVoxel;
		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, defaultVoxel);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		warp = ManipulationEngine_CPU_PVA_Warp::Inst().ReadVoxel(&scene2, coordinate);
		warp.warp_update += Vector3f(0.1);
		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene4, tolerance));

		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, defaultVoxel);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> warp_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x - 1);

	const int modifiedWarpCount = 120;

	singleVoxelTests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iWarp = 0; iWarp < modifiedWarpCount; iWarp++) {
		WarpVoxel warp;
		Vector3f framewise_warp(warp_distribution(generator), warp_distribution(generator), warp_distribution(generator));
		warp.warp_update = framewise_warp;

		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene1, coordinate, warp);
		ManipulationEngine_CPU_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene3, coordinate, warp);
		ManipulationEngine_CPU_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
	}

	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene3, tolerance));

	singleVoxelTests();
}