//  ================================================================
//  Created by Gregory Kramida on 12/27/19.
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

#pragma once

#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../InputSource/ImageSourceEngine.h"
#include "BasicVoxelEngine.h"
#include "MultiEngine.h"
#include "DynamicSceneVoxelEngine.h"
#include "BasicSurfelEngine.h"
#include "FusionAlgorithm.h"

namespace ITMLib{

FusionAlgorithm* BuildMainEngine(const RGBD_CalibrationInformation& calib, Vector2i imgSize_rgb, Vector2i imgSize_d){
	auto& settings = configuration::get();
	auto main_engine_settings = ExtractDeferrableSerializableStructFromPtreeIfPresent<MainEngineSettings>(settings.source_tree,settings.origin);
	IndexingMethod& indexing_method = main_engine_settings.indexing_method;
	FusionAlgorithm* main_engine = nullptr;

	switch (main_engine_settings.library_mode) {
		case LIBMODE_BASIC:
			switch (indexing_method) {
				case INDEX_HASH:
					main_engine = new BasicVoxelEngine<TSDFVoxel, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case INDEX_ARRAY:
					main_engine = new BasicVoxelEngine<TSDFVoxel, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		case LIBMODE_BASIC_SURFELS:
			main_engine = new BasicSurfelEngine<SurfelT>(calib, imgSize_rgb, imgSize_d);
			break;
		case LIBMODE_LOOPCLOSURE:
			switch (indexing_method) {
				case INDEX_HASH:
					main_engine = new MultiEngine<TSDFVoxel, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case INDEX_ARRAY:
					main_engine = new MultiEngine<TSDFVoxel, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		case LIBMODE_DYNAMIC:
			switch (indexing_method) {
				case INDEX_HASH:
					main_engine = new DynamicSceneVoxelEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash>(calib, imgSize_rgb, imgSize_d);
					break;
				case INDEX_ARRAY:
					main_engine = new DynamicSceneVoxelEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray>(calib, imgSize_rgb, imgSize_d);
					break;
			}
			break;
		default:
			throw std::runtime_error("Unsupported library mode!");
	}
	return main_engine;
}

} // namespace ITMLib

