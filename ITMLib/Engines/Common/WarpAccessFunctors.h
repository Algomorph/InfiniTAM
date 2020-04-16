//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "../../Utils/Math.h"
#include "../../Utils/WarpType.h"

// region ===================================== VOXEL LOOKUPS ==========================================================
namespace ITMLib{
namespace  SpecializedWarpAccess {

template<typename TVoxel, bool hasCumulativeWarp>
struct CumulativeWarpAccessStaticFunctor;

template<typename TWarp>
struct CumulativeWarpAccessStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.cumulative_warp;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.cumulative_warp;
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		voxel.cumulative_warp = warp;
	}
};

template<typename TWarp>
struct CumulativeWarpAccessStaticFunctor<TWarp, false> {
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
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		DEVICE_ASSERT(false);
	}
};

template<typename TWarp, bool hasFramewiseWarp>
struct FramewiseWarpAccessStaticFunctor;

template<typename TWarp>
struct FramewiseWarpAccessStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.framewise_warp;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.framewise_warp;
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		voxel.framewise_warp = warp;
	}
};

template<typename TWarp>
struct FramewiseWarpAccessStaticFunctor<TWarp, false> {
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
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		DEVICE_ASSERT(false);
	}
};

template <typename TWarp, bool hasWarpUpdate>
struct WarpUpdateAccessStaticFunctor;

template<typename TWarp>
struct WarpUpdateAccessStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarp& voxel, const Vector3i& position) {
		return position.toFloat() + voxel.warp_update;
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarp& voxel){
		return voxel.warp_update;
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		voxel.warp_update = warp;
	}
};


template<typename TWarp>
struct WarpUpdateAccessStaticFunctor<TWarp, false> {
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
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarp& voxel, Vector3f warp){
		DEVICE_ASSERT(false);
	}
};


}//namespace SpecializedWarpAccess

template<typename TWarpVoxel, WarpType TWarpType>
struct WarpAccessStaticFunctor;

template<typename TWarpVoxel>
struct WarpAccessStaticFunctor<TWarpVoxel, WarpType::WARP_CUMULATIVE>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarpVoxel& voxel, const Vector3i& position){
		return SpecializedWarpAccess::CumulativeWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>::GetWarpedPosition(voxel, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarpVoxel& voxel) {
		return SpecializedWarpAccess::CumulativeWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>::GetWarp(voxel);
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarpVoxel& voxel, Vector3f warp){
		SpecializedWarpAccess::CumulativeWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>::SetWarp(voxel, warp);
	}
};

template<typename TWarpVoxel>
struct WarpAccessStaticFunctor<TWarpVoxel, WarpType::WARP_FRAMEWISE>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarpVoxel& warp, const Vector3i& position){
		return SpecializedWarpAccess::FramewiseWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasFramewiseWarp>::GetWarpedPosition(warp, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarpVoxel& voxel) {
		return SpecializedWarpAccess::FramewiseWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasFramewiseWarp>::GetWarp(voxel);
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarpVoxel& voxel, Vector3f warp){
		SpecializedWarpAccess::FramewiseWarpAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasFramewiseWarp>::SetWarp(voxel, warp);
	}
};

template<typename TWarpVoxel>
struct WarpAccessStaticFunctor<TWarpVoxel, WarpType::WARP_UPDATE>{
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarpedPosition(const TWarpVoxel& voxel, const Vector3i& position){
		return SpecializedWarpAccess::WarpUpdateAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasWarpUpdate>::GetWarpedPosition(voxel, position);
	}
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetWarp(const TWarpVoxel& voxel) {
		return SpecializedWarpAccess::WarpUpdateAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasWarpUpdate>::GetWarp(voxel);
	}
	_CPU_AND_GPU_CODE_
	static inline void SetWarp(TWarpVoxel& voxel, Vector3f warp){
		SpecializedWarpAccess::WarpUpdateAccessStaticFunctor<TWarpVoxel, TWarpVoxel::hasWarpUpdate>::SetWarp(voxel, warp);
	}
};


}//namespace ITMLib
// endregion ===========================================================================================================


