//  ================================================================
//  Created by Gregory Kramida on 12/3/19.
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

#include "VoxelVolumeComparison_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "VoxelVolumeComparison_CUDA.h"
#endif

namespace ITMLib {

//TODO: remove DRY violation on CUDA/CPU selection by using an auxiliary function templated on the comparison operation to preform.
/**
 * \brief Determine if every pair of corresponding voxels within the two voxel volumes is within the provided tolerance
 * of each other.
 * \details Voxel correspondence between the two volumes is determined by coinciding spatial location within both volumes.
 * The two scenes may use different indices.
 * \tparam TVoxel voxel type
 * \tparam TIndexA type of index for the first volume
 * \tparam TIndexB type of index for the second volume
 * \tparam ToleranceType type of the tolerance metric
 * \param a the first voxel volume
 * \param b the second voxel volume
 * \param tolerance (absolute) difference bound for each quantitative value in each voxel
 * \return true if scene content matches (to within specified tolerance), false otherwise
 */
template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool ContentAlmostEqual(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, ToleranceType tolerance,
                        MemoryDeviceType memory_type) {
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return contentAlmostEqual_CPU(a, b, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return contentAlmostEqual_CUDA(a, b, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
			return false;
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}

}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool ContentAlmostEqual_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                ToleranceType tolerance, MemoryDeviceType memory_type){
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return contentAlmostEqual_CPU_Verbose(a, b, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return contentAlmostEqual_CUDA_Verbose(a, b, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
			return false;
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}
}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool
ContentForFlagsAlmostEqual(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b, VoxelFlags flags,
                           ToleranceType tolerance, MemoryDeviceType memory_type){
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return contentForFlagsAlmostEqual_CPU(a, b, flags, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return contentForFlagsAlmostEqual_CUDA(a, b, flags, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
			return false;
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}
}

template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool ContentForFlagsAlmostEqual_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                        VoxelFlags flags, ToleranceType tolerance, MemoryDeviceType memory_type){
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return contentForFlagsAlmostEqual_CPU_Verbose(a, b, flags, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return contentForFlagsAlmostEqual_CUDA_Verbose(a, b, flags, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
			return false;
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}
}

/**
 * \brief Determine if every pair of corresponding voxels within the two voxel volumes is within the provided tolerance
 * of each other, ignoring areas that were not allocated in either one of the scenes (if present).
 * \details Voxel correspondence between the two volumes is determined by coinciding spatial location within both volumes.
 * The two scenes may use different indices. The areas with no allocated voxels are ignored, even if there are altered
 * voxels within the other volume. This includes, for a VBH<-->PVA volume pair, areas that are present
 * (and, potentially, altered) in PVA but do not have allocated in VBH, and, likewise, allocated
 * (and perhaps altered) voxel blocks in VBH that fall outside of the extent of the PVA.
 * \tparam TVoxel voxel type
 * \tparam TIndexA type of index for the first volume
 * \tparam TIndexB type of index for the second volume
 * \tparam ToleranceType type of the tolerance metric
 * \param a the first voxel volume
 * \param b the second voxel volume
 * \param tolerance (absolute) difference bound for each quantitative value in each voxel
 * \return true if scene content for allocated areas in both volumes matches (to within specified tolerance), false otherwise
 */
template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool AllocatedContentAlmostEqual(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                 ToleranceType tolerance, MemoryDeviceType memory_type){
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return allocatedContentAlmostEqual_CPU(a, b, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return allocatedContentAlmostEqual_CUDA(a, b, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}
}
template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool AllocatedContentAlmostEqual_Verbose(VoxelVolume<TVoxel, TIndexA>* a, VoxelVolume<TVoxel, TIndexB>* b,
                                         ToleranceType tolerance, MemoryDeviceType memory_type){
	switch (memory_type) {
		case MEMORYDEVICE_CPU:
			return allocatedContentAlmostEqual_CPU_Verbose(a, b, tolerance);
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			printf("WARNING: trying to run CUDA volume comparison while compiled without CUDA\n");
			return false;
#else
			return allocatedContentAlmostEqual_CUDA_Verbose(a, b, tolerance);
#endif
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
#error METAL SUPPORT NOT FULLY IMPLEMENTED
#else
			printf("WARNING: trying to run METAL volume comparison while compiled without METAL\n");
			return false;
#endif
		case MEMORYDEVICE_NONE:
			DIEWITHEXCEPTION_REPORTLOCATION("memory_type cannot be MEMORYDEVICE_NONE for comparison.");
			return false;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported memory device.");
			return false;
	}
}
} // namespace ITMLib