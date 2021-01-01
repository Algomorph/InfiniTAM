//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 11/27/20.
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
#define BOOST_TEST_MODULE RigidAlignment

#include "TestUtilities/GenericTestModuleHeader.h"
#include "TestUtilities/RigidTrackerPresets.h"
#include "../ITMLib/CameraTrackers/CameraTrackerFactory.h"
#include "../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ITMLib/Objects/Misc/IMUCalibrator.h"
#include "../ITMLib/Engines/Raycasting/RaycastingEngineFactory.h"
#include "../ORUtils/MathTypePersistence/MathTypePersistence.h"
#include "../ORUtils/MemoryBlockPersistence/MemoryBlockPersistence.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct TestEnvironment {
public:
	View* view_teddy_frame116;
	CameraTrackingState tracking_state_color_and_depth;
	CameraTrackingState tracking_state_depth_only;
	IMUCalibrator* imu_calibrator;
	ImageProcessingEngineInterface* image_processing_engine;
private:
	VoxelVolume<TSDFVoxel_f_rgb, TIndex> volume_teddy_frame115;
	RaycastingEngineBase<TSDFVoxel_f_rgb, TIndex>* raycasting_engine;
public:
	TestEnvironment() :

			view_teddy_frame116(nullptr),
			// tracking state initialized with identity for camera matrix
			tracking_state_color_and_depth(teddy::frame_image_size, TMemoryDeviceType),
			tracking_state_depth_only(teddy::frame_image_size, TMemoryDeviceType),
			imu_calibrator(new ITMIMUCalibrator_iPad()),
			image_processing_engine(ImageProcessingEngineFactory::Build(TMemoryDeviceType)),
			volume_teddy_frame115(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
			                      teddy::PartialInitializationParameters<TIndex>()),
			// the rendering engine will generate the point cloud inside tracking_state_color_and_depth
			raycasting_engine(ITMLib::RaycastingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType)) {

		// A little counter-intuitive, but we'll use the same view for frame_115 at first
		// because of the weird way view memory is currently managed.
		UpdateView(&view_teddy_frame116,
		           teddy::frame_115_depth_path.ToString(),
		           teddy::frame_115_color_path.ToString(),
		           teddy::calibration_path.ToString(),
		           TMemoryDeviceType);

		// the rendering engine will generate the point cloud
		raycasting_engine = ITMLib::RaycastingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType);

		// we need the volume of the frame 115 to generate the point cloud for the tracker
		volume_teddy_frame115.Reset();
		volume_teddy_frame115.LoadFromDisk(teddy::PartialVolume115Path<TIndex>());


		// we need a dud rendering state also for the point cloud
		RenderState render_state(teddy::frame_image_size,
		                         teddy::DefaultVolumeParameters().near_clipping_distance,
		                         teddy::DefaultVolumeParameters().far_clipping_distance,
		                         TMemoryDeviceType);

		ORUtils::SE3Pose pose_rgb(
				view_teddy_frame116->calibration_information.trafo_rgb_to_depth.calib_inv * tracking_state_color_and_depth.pose_d->GetM());
		raycasting_engine->CreateExpectedDepths(&volume_teddy_frame115, &pose_rgb, &(view_teddy_frame116->calibration_information.intrinsics_rgb),
		                                        &render_state);
		raycasting_engine->CreatePointCloud(&volume_teddy_frame115, view_teddy_frame116, &tracking_state_color_and_depth, &render_state);
		raycasting_engine->CreateExpectedDepths(&volume_teddy_frame115, tracking_state_depth_only.pose_d,
		                                        &(view_teddy_frame116->calibration_information.intrinsics_d), &render_state);
		raycasting_engine->CreateICPMaps(&volume_teddy_frame115, view_teddy_frame116, &tracking_state_depth_only, &render_state);

		UpdateView(&view_teddy_frame116,
		           teddy::frame_116_depth_path.ToString(),
		           teddy::frame_116_color_path.ToString(),
		           teddy::calibration_path.ToString(),
		           TMemoryDeviceType);
	}

	void ResetTrackingState() {
		this->tracking_state_color_and_depth.pose_d->SetM(Matrix4f::Identity());
		this->tracking_state_depth_only.pose_d->SetM(Matrix4f::Identity());
	}

	~TestEnvironment() {
		delete view_teddy_frame116;
		delete raycasting_engine;
		delete image_processing_engine;
		delete imu_calibrator;
	}
};


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericRigidTrackerTest(const std::string& preset, TestEnvironment<TIndex, TMemoryDeviceType>& environment, float absolute_tolerance) {
	BOOST_TEST_MESSAGE("Using preset: \n" << preset);


	CameraTracker* tracker = CameraTrackerFactory::Instance().Make(
			TMemoryDeviceType, preset.c_str(), teddy::frame_image_size, teddy::frame_image_size,
			environment.image_processing_engine, environment.imu_calibrator,
			teddy::DefaultVolumeParameters()
	);


	//__DEBUG
	// PointCloud point_cloud_gt(teddy::frame_image_size, TMemoryDeviceType);
	// View view_gt(environment.view_teddy_frame116->calibration_information, teddy::frame_image_size, teddy::frame_image_size,
	//              TMemoryDeviceType == MEMORYDEVICE_CUDA);
	// ORUtils::IStreamWrapper debug_reader(std::string(test::generated_arrays_directory) + "/debug.dat");
	// debug_reader >> point_cloud_gt;
	// debug_reader >> view_gt;
	// BOOST_REQUIRE(point_cloud_gt == *environment.tracking_state_color_and_depth.point_cloud);
	// BOOST_REQUIRE(view_gt == *environment.view_teddy_frame116);

	CameraTrackingState* tracking_state_to_use = nullptr;
	if (tracker->requiresColourRendering()) {
		tracking_state_to_use = &environment.tracking_state_color_and_depth;
	} else {
		tracking_state_to_use = &environment.tracking_state_depth_only;
	}

	tracker->TrackCamera(tracking_state_to_use, environment.view_teddy_frame116);


	const std::string& matrix_filename = test::matrix_file_name_by_preset.at(preset);
	std::string matrix_path = test::generated_matrix_directory.ToString() + "/" + matrix_filename;
	ORUtils::IStreamWrapper matrix_reader(matrix_path);
	Matrix4f depth_matrix_gt;
	matrix_reader >> depth_matrix_gt;

	// __DEBUG
	std::cout << matrix_filename << std::endl << std::endl;
	std::cout << depth_matrix_gt << std::endl;
	std::cout << tracking_state_to_use->pose_d->GetM() << std::endl;

	BOOST_REQUIRE(AlmostEqual(depth_matrix_gt, tracking_state_to_use->pose_d->GetM(), absolute_tolerance));
	environment.ResetTrackingState();

	delete tracker;
}

typedef TestEnvironment<VoxelBlockHash, MEMORYDEVICE_CPU> environment_VBH_CPU;

BOOST_FIXTURE_TEST_CASE(Test_RgbTracker_CPU_VBH, environment_VBH_CPU) {
	float absolute_tolerance = 1.0e-2;

	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_t, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_r, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_b, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrbb, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrrbb, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrrbrb, *this, absolute_tolerance);
}

BOOST_FIXTURE_TEST_CASE(Test_ExtendedTracker_CPU_VBH, environment_VBH_CPU) {
	float absolute_tolerance = 1.0e-3;
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::extended_tracker_preset1, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::extended_tracker_preset2, *this, absolute_tolerance);
}

BOOST_FIXTURE_TEST_CASE(Test_DepthTracker_CPU_VBH, environment_VBH_CPU) {
	float absolute_tolerance = 1.0e-3;
	for (auto& preset : test::depth_tracker_presets) {
		GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(preset, *this, absolute_tolerance);
	}
}

#ifndef COMPILE_WITHOUT_CUDA
typedef TestEnvironment<VoxelBlockHash, MEMORYDEVICE_CUDA> environment_VBH_CUDA;

BOOST_FIXTURE_TEST_CASE(Test_RgbTracker_CUDA_VBH, environment_VBH_CUDA) {

	// There is very high deviation for translation here -- may be related to z-motion,
	// but somehow also related to GPU implementation vs. CPU impementation
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::rgb_tracker_preset_t, *this, 0.05);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::rgb_tracker_preset_r, *this, 0.02);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::rgb_tracker_preset_b, *this, 0.10);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::rgb_tracker_preset_rrbb, *this, 0.16);
	// The standard deviation for the GPU implementation with rrrbb is only about 0.014, but the difference from CPU is dramatic
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::rgb_tracker_preset_rrrbb, *this, 0.16);

}

BOOST_FIXTURE_TEST_CASE(Test_ExtendedTracker_CUDA_VBH, environment_VBH_CUDA) {
	float absolute_tolerance = 1.0e-3;
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::extended_tracker_preset1, *this, absolute_tolerance);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(test::extended_tracker_preset2, *this, absolute_tolerance);
}

BOOST_FIXTURE_TEST_CASE(Test_DepthTracker_CUDA_VBH, environment_VBH_CUDA) {
	float absolute_tolerance = 1.0e-3;
	for (auto& preset : test::depth_tracker_presets) {
		GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(preset, *this, absolute_tolerance);
	}
}
#endif