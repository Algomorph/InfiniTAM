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
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../ORUtils/MathTypePersistence/MathTypePersistence.h"
#include "../ORUtils/MemoryBlockPersistence/MemoryBlockPersistence.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct TestEnvironment {
public:
	View* view_teddy_frame116;
	CameraTrackingState tracking_state;
	IMUCalibrator* imu_calibrator;
	ImageProcessingEngineInterface* image_processing_engine;
private:
	VoxelVolume<TSDFVoxel_f_rgb, TIndex> volume_teddy_frame115;
	RenderingEngineBase<TSDFVoxel_f_rgb, TIndex>* rendering_engine;
public:
	TestEnvironment() :

			view_teddy_frame116(nullptr),
			// tracking state initialized with identity for camera matrix
			tracking_state(teddy::frame_image_size, TMemoryDeviceType),
			imu_calibrator(new ITMIMUCalibrator_iPad()),
			image_processing_engine(ImageProcessingEngineFactory::Build(TMemoryDeviceType)),
			volume_teddy_frame115(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
			                      teddy::InitializationParameters<TIndex>()),
			// the rendering engine will generate the point cloud inside tracking_state
			rendering_engine(ITMLib::RenderingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType)) {

		// A little counter-intuitive, but we'll use the same view for frame_115 at first
		// because of the weird way view memory is currently managed.
		UpdateView(&view_teddy_frame116,
		           teddy::frame_115_depth_path.ToString(),
		           teddy::frame_115_color_path.ToString(),
		           teddy::calibration_path.ToString(),
		           TMemoryDeviceType);

		// the rendering engine will generate the point cloud
		rendering_engine = ITMLib::RenderingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType);

		// we need the volume of the frame 115 to generate the point cloud for the tracker
		volume_teddy_frame115.Reset();
		volume_teddy_frame115.LoadFromDisk(teddy::Volume115Path<TIndex>());


		// we need a dud rendering state also for the point cloud
		RenderState render_state(teddy::frame_image_size,
		                         teddy::DefaultVolumeParameters().near_clipping_distance,
		                         teddy::DefaultVolumeParameters().far_clipping_distance,
		                         TMemoryDeviceType);

		rendering_engine->CreatePointCloud(&volume_teddy_frame115, view_teddy_frame116, &tracking_state, &render_state);
		UpdateView(&view_teddy_frame116,
		           teddy::frame_116_depth_path.ToString(),
		           teddy::frame_116_color_path.ToString(),
		           teddy::calibration_path.ToString(),
		           TMemoryDeviceType);
	}

	void ResetTrackingState() {
		this->tracking_state.pose_d->SetM(Matrix4f::Identity());
	}

	~TestEnvironment() {
		delete view_teddy_frame116;
		delete rendering_engine;
		delete image_processing_engine;
		delete imu_calibrator;
	}
};


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericRigidTrackerTest(const std::string& preset, TestEnvironment<TIndex, TMemoryDeviceType>& environment) {
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
	// BOOST_REQUIRE(point_cloud_gt == *environment.tracking_state.point_cloud);
	// BOOST_REQUIRE(view_gt == *environment.view_teddy_frame116);


	tracker->TrackCamera(&environment.tracking_state, environment.view_teddy_frame116);

	const std::string& matrix_filename = test::matrix_file_name_by_preset.at(preset);
	std::string matrix_path = test::generated_matrix_directory.ToString() + "/" + matrix_filename;
	ORUtils::IStreamWrapper matrix_reader(matrix_path);
	Matrix4f depth_matrix_gt;
	matrix_reader >> depth_matrix_gt;

	// __DEBUG
	std::cout << matrix_filename << std::endl << std::endl;
	std::cout << depth_matrix_gt << std::endl;
	std::cout << environment.tracking_state.pose_d->GetM() << std::endl;

	BOOST_REQUIRE(AlmostEqual(depth_matrix_gt, environment.tracking_state.pose_d->GetM(), 1.0e-3));
	environment.ResetTrackingState();

	delete tracker;
}

typedef TestEnvironment<VoxelBlockHash, MEMORYDEVICE_CPU> environment_VBH_CPU;

BOOST_FIXTURE_TEST_CASE(Test_RgbTracker_CPU_VBH, environment_VBH_CPU) {
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrbb, *this);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrbrb, *this);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::rgb_tracker_preset_rrrbb, *this);
}

BOOST_FIXTURE_TEST_CASE(Test_ExtendedTracker_CPU_VBH, environment_VBH_CPU) {
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::extended_tracker_preset1, *this);
	GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(test::extended_tracker_preset2, *this);
}

BOOST_FIXTURE_TEST_CASE(Test_DepthTracker_CPU_VBH, environment_VBH_CPU){
	for(auto& preset : test::depth_tracker_presets){
		GenericRigidTrackerTest<VoxelBlockHash, MEMORYDEVICE_CPU>(preset, *this);
	}
}