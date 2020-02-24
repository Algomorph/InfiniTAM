//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include <random>

#include "TestUtils.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
#include "../ORUtils/FileUtils.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#endif


using namespace ITMLib;

template<class TVoxel, class TIndex>
void GenerateTestVolume_CPU(VoxelVolume<TVoxel, TIndex>* volume) {
	EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ResetVolume(volume);
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxelsZ = 16;
	int surfaceSizeVoxelsY = 64;

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {
		float sdfMagnitude = 0.0f + static_cast<float>(iVoxelAcrossBand) * (1.0f / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		simulateVoxelAlteration(voxelNeg, sdfMagnitude);
		simulateVoxelAlteration(voxelPos, -sdfMagnitude);

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().SetVoxel(volume, Vector3i(xPos, y, z), voxelPos);
				EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().SetVoxel(volume, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}

#ifndef COMPILE_WITHOUT_CUDA

template<class TVoxel, class TIndex>
void GenerateTestVolume_CUDA(VoxelVolume<TVoxel, TIndex>* volume) {
	volume->Reset();
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxelsZ = 16;
	int surfaceSizeVoxelsY = 64;

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {
		float sdfMagnitude = 0.0f + static_cast<float>(iVoxelAcrossBand) * (1.0f / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		simulateVoxelAlteration(voxelNeg, sdfMagnitude);
		simulateVoxelAlteration(voxelPos, -sdfMagnitude);

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst().SetVoxel(volume, Vector3i(xPos, y, z), voxelPos);
				EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst().SetVoxel(volume, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}

#endif

template<bool hasSemanticInformation, typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor;

template<typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static
	void run(TVoxel& voxel) {
		if (voxel.sdf > -1.0f && voxel.sdf < 1.0f) {
			voxel.flags = VoxelFlags::VOXEL_NONTRUNCATED;
		} else {
			voxel.flags = VoxelFlags::VOXEL_TRUNCATED;
		}
	}
};

template<typename TVoxel>
struct HandleSDFBasedFlagsAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void run(TVoxel& voxel) {}
};

//judiciously ignore clang warnings about something from stdlib throwing uncatchable exceptions in default constructors
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
namespace internal {
std::random_device random_device;
std::mt19937 generator(random_device());
std::uniform_real_distribution<float> floatDistribution(-1.0f, 1.0f);
}
#pragma clang diagnostic pop

template<bool hasSDFInformation, typename TVoxel>
struct HandleSDFAlterationFunctor;

template<typename TVoxel>
struct HandleSDFAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {
		voxel.sdf = TVoxel::floatToValue(value);
		voxel.w_depth += 1;
		HandleSDFBasedFlagsAlterationFunctor<TVoxel::hasSemanticInformation, TVoxel>::run(voxel);
	}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {
		float value = internal::floatDistribution(internal::generator);
	}
};

template<typename TVoxel>
struct HandleSDFAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {}
};

template<bool hasFramewiseWarp, typename TVoxel>
struct HandleFramewiseWarpAlterationFunctor;

template<typename TVoxel>
struct HandleFramewiseWarpAlterationFunctor<true, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, Vector3f value) {
		voxel.framewise_warp = value;
	}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {
		Vector3f value(internal::floatDistribution(internal::generator),
		               internal::floatDistribution(internal::generator),
		               internal::floatDistribution(internal::generator));
	}
};

template<typename TVoxel>
struct HandleFramewiseWarpAlterationFunctor<false, TVoxel> {
	_CPU_AND_GPU_CODE_
	inline static void setValue(TVoxel& voxel, float value) {}

	_CPU_AND_GPU_CODE_
	inline static void setRandom(TVoxel& voxel) {}
};


template<typename TVoxel>
void simulateVoxelAlteration(TVoxel& voxel, float newSdfValue) {
	HandleSDFAlterationFunctor<TVoxel::hasSDFInformation, TVoxel>::setValue(voxel, newSdfValue);
}

template<typename TVoxel>
void simulateRandomVoxelAlteration(TVoxel& voxel) {
	HandleSDFAlterationFunctor<TVoxel::hasSDFInformation, TVoxel>::setRandom(voxel);
	HandleFramewiseWarpAlterationFunctor<TVoxel::hasFramewiseWarp, TVoxel>::setRandom(voxel);
}

// FIXME: see TODO in header
//template<typename TVoxelA, typename TIndex>
//ITMVoxelVolume<TVoxelA, TIndex> loadVolume (const std::string& path, MemoryDeviceType memoryDeviceType,
//                    typename TIndex::InitializationParameters initializationParameters, configuration::SwappingMode swapping_mode){
//	Configuration& settings = configuration::get();
//	ITMVoxelVolume<TVoxelA, TIndex> scene(&settings.general_voxel_volume_parameters,
//	                                              swapping_mode,
//	                                              memoryDeviceType,initializationParameters);
//	PrepareVoxelVolumeForLoading(&scene, memoryDeviceType);
//	scene.LoadFromDirectory(path);
//	return scene;
//};

template<typename TVoxel, typename TIndex>
void loadVolume(VoxelVolume<TVoxel, TIndex>** volume, const std::string& path, MemoryDeviceType memoryDeviceType,
                typename TIndex::InitializationParameters initializationParameters,
                configuration::SwappingMode swappingMode) {
	configuration::Configuration& settings = configuration::get();
	(*volume) = new VoxelVolume<TVoxel, TIndex>(&settings.general_voxel_volume_parameters,
	                                            swappingMode,
	                                            memoryDeviceType, initializationParameters);
	PrepareVoxelVolumeForLoading(*volume);
	(*volume)->LoadFromDirectory(path);
}


template<typename TVoxel, typename TIndex>
void initializeVolume(VoxelVolume<TVoxel, TIndex>** volume,
                      typename TIndex::InitializationParameters initializationParameters, MemoryDeviceType memoryDevice,
                      configuration::SwappingMode swappingMode) {
	(*volume) = new VoxelVolume<TVoxel, TIndex>(memoryDevice, initializationParameters);
	(*volume)->Reset();
}


template<typename TVoxel, typename TIndex>
void buildSdfVolumeFromImage_NearSurfaceAllocation(VoxelVolume<TVoxel, TIndex>** volume,
                                                   View** view,
                                                   const std::string& depth_path,
                                                   const std::string& color_path,
                                                   const std::string& mask_path,
                                                   const std::string& calibration_path,
                                                   MemoryDeviceType memory_device,
                                                   typename TIndex::InitializationParameters initializationParameters,
                                                   configuration::SwappingMode swappingMode
) {

	// region ================================= CONSTRUCT VIEW =========================================================
	Vector2i imageSize(640, 480);
	updateView(view, depth_path, color_path, mask_path, calibration_path, memory_device);
	initializeVolume(volume, initializationParameters, memory_device, swappingMode);
	(*volume) = new VoxelVolume<TVoxel, TIndex>(&configuration::get().general_voxel_volume_parameters, swappingMode,
	                                            memory_device, initializationParameters);
	(*volume)->Reset();
	RenderState renderState(imageSize, configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                        configuration::get().general_voxel_volume_parameters.far_clipping_distance, memory_device);
	CameraTrackingState tracking_state(imageSize, memory_device);

	DepthFusionEngineInterface<TVoxel, WarpVoxel, TIndex>* depth_fusion_engine =
			DepthFusionEngineFactory
			::Build<TVoxel, WarpVoxel, TIndex>(memory_device);

	IndexingEngineInterface<TSDFVoxel, TIndex>* indexing_engine = IndexingEngineFactory::Build<TVoxel, TIndex>(
			memory_device);
	indexing_engine->AllocateNearSurface(*volume, *view, &tracking_state, false, false);
	depth_fusion_engine->GenerateTsdfVolumeFromSurface(*volume, *view, &tracking_state);

	delete depth_fusion_engine;
	delete indexing_engine;
}

template<typename TVoxel, typename TIndex>
void buildSdfVolumeFromImage_NearSurfaceAllocation(VoxelVolume<TVoxel, TIndex>** volume,
                                                   const std::string& depth_path, const std::string& color_path,
                                                   const std::string& mask_path,
                                                   const std::string& calibration_path,
                                                   MemoryDeviceType memory_device,
                                                   typename TIndex::InitializationParameters initialization_parameters,
                                                   configuration::SwappingMode swapping_mode) {

	// region ================================= CONSTRUCT VIEW =========================================================

	View* view = nullptr;
	buildSdfVolumeFromImage_NearSurfaceAllocation(volume, &view,
	                                              depth_path,
	                                              color_path,
	                                              mask_path,
	                                              calibration_path,
	                                              memory_device,
	                                              initialization_parameters, swapping_mode);
	delete view;
}

template<typename TVoxel, typename TIndex>
void buildSdfVolumeFromImage_SurfaceSpanAllocation(VoxelVolume<TVoxel, TIndex>** volume1,
                                                   VoxelVolume<TVoxel, TIndex>** volume2,
                                                   View** view,
                                                   const std::string& depth1_path,
                                                   const std::string& color1_path,
                                                   const std::string& mask1_path,
                                                   const std::string& depth2_path,
                                                   const std::string& color2_path,
                                                   const std::string& mask2_path,
                                                   const std::string& calibration_path,
                                                   MemoryDeviceType memory_device,
                                                   typename TIndex::InitializationParameters initialization_parameters,
                                                   configuration::SwappingMode swapping_mode
) {

	// region ================================= CONSTRUCT VIEW =========================================================
	updateView(view, depth1_path, color1_path, mask1_path, calibration_path, memory_device);
	initializeVolume(volume1, initialization_parameters, memory_device, swapping_mode);
	(*volume1) = new VoxelVolume<TVoxel, TIndex>(memory_device, initialization_parameters);
	(*volume1)->Reset();
	(*volume2) = new VoxelVolume<TVoxel, TIndex>(memory_device, initialization_parameters);
	(*volume2)->Reset();

	RenderState render_state((*view)->depth->noDims,
	                         configuration::get().general_voxel_volume_parameters.near_clipping_distance,
	                         configuration::get().general_voxel_volume_parameters.far_clipping_distance, memory_device);
	CameraTrackingState tracking_state((*view)->depth->noDims, memory_device);
	IndexingEngineInterface<TSDFVoxel, TIndex>* indexing_engine = IndexingEngineFactory::Build<TVoxel, TIndex>(
			memory_device);
	DepthFusionEngineInterface<TVoxel, WarpVoxel, TIndex>* depth_fusion_engine =
			DepthFusionEngineFactory
			::Build<TVoxel, WarpVoxel, TIndex>(memory_device);

	indexing_engine->AllocateNearSurface(*volume1,*view,&tracking_state, false, false);
	depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(*volume1, *view, &tracking_state);

	VisualizationEngine<TSDFVoxel, TIndex>* visualization_engine =
			VisualizationEngineFactory::MakeVisualizationEngine<TSDFVoxel, TIndex>(memory_device);
	visualization_engine->CreateICPMaps(*volume1, *view, &tracking_state, &render_state);

	updateView(view, depth2_path, color2_path, mask2_path, calibration_path, memory_device);

	indexing_engine->AllocateNearAndBetweenTwoSurfaces(*volume2, *view, &tracking_state);
	depth_fusion_engine->IntegrateDepthImageIntoTsdfVolume(*volume2, *view, &tracking_state);

	delete indexing_engine;
	delete depth_fusion_engine;
}


template<typename TVoxel, typename TIndex>
void buildSdfVolumeFromImage_SurfaceSpanAllocation(VoxelVolume<TVoxel, TIndex>** volume1,
                                                   VoxelVolume<TVoxel, TIndex>** volume2,
                                                   const std::string& depth1_path,
                                                   const std::string& color1_path,
                                                   const std::string& mask1_path,
                                                   const std::string& depth2_path,
                                                   const std::string& color2_path,
                                                   const std::string& mask2_path,
                                                   const std::string& calibration_path,
                                                   MemoryDeviceType memory_device,
                                                   typename TIndex::InitializationParameters initialization_parameters,
                                                   configuration::SwappingMode swapping_mode)
{
	View* view = nullptr;
	buildSdfVolumeFromImage_SurfaceSpanAllocation(volume1, volume2, &view,
	                                              depth1_path,
	                                              color1_path,
	                                              mask1_path,
	                                              depth2_path,
	                                              color2_path,
	                                              mask2_path,
	                                              calibration_path,
	                                              memory_device,
	                                              initialization_parameters, swapping_mode);
}