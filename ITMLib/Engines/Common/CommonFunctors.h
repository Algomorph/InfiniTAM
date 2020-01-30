//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMMath.h"
#include "WarpType.h"

// region ===================================== VOXEL LOOKUPS ==========================================================
namespace ITMLib{
namespace  SpecializedWarpLookups {

template<typename TVoxel, bool hasCumulativeWarp>
struct LookupBasedOnCumulativeWarpStaticFunctor;

template<typename TWarp>
struct LookupBasedOnCumulativeWarpStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.warp;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.warp;
	}
};

template<typename TWarp>
struct LookupBasedOnCumulativeWarpStaticFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		//TODO: after proper CUDA error-handling is in place, reinstate the "exception", likewise for the other 2 "error" functor versions
		//DIEWITHEXCEPTION_REPORTLOCATION("Attempting to use cumulative warps with voxel type that doesn't have them.");
		DEVICE_ASSERT(false);
		return position.toFloat();
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		DEVICE_ASSERT(false);
		return Vector3f(0.0f);
	}
};

template<typename TWarp, bool hasFramewiseWarp>
struct LookupBasedOnFramewiseWarpStaticFunctor;

template<typename TWarp>
struct LookupBasedOnFramewiseWarpStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.framewise_warp;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.framewise_warp;
	}
};

template<typename TWarp>
struct LookupBasedOnFramewiseWarpStaticFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		//DIEWITHEXCEPTION_REPORTLOCATION("Attempting to use flow warps with voxel type that doesn't have them.");
		DEVICE_ASSERT(false);
		return position.toFloat();
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		DEVICE_ASSERT(false);
		return Vector3f(0.0f);
	}
};

template <typename TWarp, bool hasWarpUpdate>
struct LookupBasedOnWarpUpdateStaticFunctor;

template<typename TWarp>
struct LookupBasedOnWarpUpdateStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.warp_update;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.warp_update;
	}

};


template<typename TWarp>
struct LookupBasedOnWarpUpdateStaticFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		//DIEWITHEXCEPTION_REPORTLOCATION("Attempting to use warp updates with voxel type that doesn't have them.");
		DEVICE_ASSERT(false);
		return position.toFloat();
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		DEVICE_ASSERT(false);
		return Vector3f(0.0f);
	}
};


}//namespace SpecializedWarpLookups

template<typename TWarp, WarpType TWarpType>
struct WarpVoxelStaticFunctor;

template<typename TWarp>
struct WarpVoxelStaticFunctor<TWarp, WarpType::WARP_CUMULATIVE>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position){
		return SpecializedWarpLookups::LookupBasedOnCumulativeWarpStaticFunctor<TWarp, TWarp::hasCumulativeWarp>::GetWarpedPosition(voxel, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel) {
		return SpecializedWarpLookups::LookupBasedOnCumulativeWarpStaticFunctor<TWarp, TWarp::hasCumulativeWarp>::GetWarp(voxel);
	}
};

template<typename TWarp>
struct WarpVoxelStaticFunctor<TWarp, WarpType::WARP_FLOW>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& warp, const Vector3i& position){
		return SpecializedWarpLookups::LookupBasedOnFramewiseWarpStaticFunctor<TWarp, TWarp::hasFramewiseWarp>::GetWarpedPosition(warp, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel) {
		return SpecializedWarpLookups::LookupBasedOnFramewiseWarpStaticFunctor<TWarp, TWarp::hasFramewiseWarp>::GetWarp(voxel);
	}
};

template<typename TWarp>
struct WarpVoxelStaticFunctor<TWarp, WarpType::WARP_UPDATE>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position){
		return SpecializedWarpLookups::LookupBasedOnWarpUpdateStaticFunctor<TWarp, TWarp::hasWarpUpdate>::GetWarpedPosition(voxel, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel) {
		return SpecializedWarpLookups::LookupBasedOnWarpUpdateStaticFunctor<TWarp, TWarp::hasWarpUpdate>::GetWarp(voxel);
	}
};


}//namespace ITMLib
// endregion ===========================================================================================================


