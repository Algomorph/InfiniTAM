// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
// _Algomorph modified code -- Copyright 2017-2020 Gregory Kramida
#pragma once

#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Utils/PixelUtils.h"
#include "../../../Utils/Geometry/Segment.h"
#include "../../../Utils/Geometry/IntersectionChecks.h"

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#include "../../../Utils/CUDAUtils.h"
#endif

_CPU_AND_GPU_CODE_ inline void
MarkForAllocationAndSetVisibilityTypeIfNotFound_Legacy_Algomorph(ITMLib::HashEntryAllocationState* hashEntryStates,
                                                                 Vector3s* hashBlockCoordinates,
                                                                 ITMLib::HashBlockVisibility* blockVisibilityTypes,
                                                                 Vector3s desiredHashBlockPosition,
                                                                 const CONSTPTR(HashEntry)* hashTable,
                                                                 bool& collisionDetected) {

	int hashCode = HashCodeFromBlockPosition(desiredHashBlockPosition);

	HashEntry hashEntry = hashTable[hashCode];

	//check if hash table contains entry
	if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
		//entry has been streamed out but is visible or in memory and visible
		blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? ITMLib::STREAMED_OUT_AND_VISIBLE
		                                                       : ITMLib::IN_MEMORY_AND_VISIBLE;
		return;
	}

	ITMLib::HashEntryAllocationState allocationState = ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST;
	if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
	{
		while (hashEntry.offset >= 1) {
			hashCode = ORDERED_LIST_SIZE + hashEntry.offset - 1;
			hashEntry = hashTable[hashCode];

			if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
				//entry has been streamed out but is visible or in memory and visible
				blockVisibilityTypes[hashCode] = (hashEntry.ptr == -1) ? ITMLib::STREAMED_OUT_AND_VISIBLE
				                                                       : ITMLib::IN_MEMORY_AND_VISIBLE;
				return;
			}
		}
		allocationState = ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST;
	}

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
	if (atomicCAS((char*) hashEntryStates + hashCode,
				  (char) ITMLib::NEEDS_NO_CHANGE,
				  (char) allocationState) != ITMLib::NEEDS_NO_CHANGE) {
		if (IS_EQUAL3(hashBlockCoordinates[hashCode], desiredHashBlockPosition)) return;
		collisionDetected = true;
	} else {
		//needs allocation
		if (allocationState == ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST)
			blockVisibilityTypes[hashCode] = ITMLib::IN_MEMORY_AND_VISIBLE; //new entry is visible
		hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
	}
#else
#if defined(WITH_OPENMP)
#pragma omp critical
#endif
	{
		if (hashEntryStates[hashCode] != ITMLib::NEEDS_NO_CHANGE) {
			collisionDetected = true;
		} else {
			//needs allocation
			hashEntryStates[hashCode] = allocationState;
			if (allocationState == ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST)
				blockVisibilityTypes[hashCode] = ITMLib::IN_MEMORY_AND_VISIBLE; //new entry is visible
			hashBlockCoordinates[hashCode] = desiredHashBlockPosition;
		}
	}
#endif
}


_CPU_AND_GPU_CODE_ inline int getIncrementCount(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}

_CPU_AND_GPU_CODE_ inline int getIncrementCountLegacy_Algomorph(Vector3s coord1, Vector3s coord2) {
	return static_cast<int>(coord1.x != coord2.x) +
	       static_cast<int>(coord1.y != coord2.y) +
	       static_cast<int>(coord1.z != coord2.z);
}

_CPU_AND_GPU_CODE_ inline void
findVoxelHashBlocksAlongSegmentLegacy_Algomorph(ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
                                                Vector3s* hash_block_coordinates,
                                                ITMLib::HashBlockVisibility* hash_block_visibility_types,
                                                const CONSTPTR(HashEntry)* hash_table,
                                                const ITMLib::Segment& segment_in_hash_blocks,
                                                bool& collision_detected) {

	// number of steps to take along the truncated SDF band
	int step_count = (int) std::ceil(2.0f * segment_in_hash_blocks.length());

	// a single stride along the sdf band segment from one step to the next
	Vector3f strideVector = segment_in_hash_blocks.direction / (float) (step_count - 1);

	Vector3s previousHashBlockPosition;
	Vector3f check_position = segment_in_hash_blocks.origin;

	//add neighbouring blocks
	for (int i = 0; i < step_count; i++) {
		//find block position at current step
		Vector3s currentHashBlockPosition = TO_SHORT_FLOOR3(check_position);
		int incrementCount;
		if (i > 0 &&
		    (incrementCount = getIncrementCountLegacy_Algomorph(currentHashBlockPosition, previousHashBlockPosition)) >
		    1) {
			if (incrementCount == 2) {
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					if (currentHashBlockPosition.values[iDirection] != previousHashBlockPosition.values[iDirection]) {
						Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
						potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
						if (ITMLib::SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
						                                               TO_FLOAT3(potentiallyMissedBlockPosition),
						                                               1.0f)) {
							MarkForAllocationAndSetVisibilityTypeIfNotFound_Legacy_Algomorph(
									hash_entry_allocation_states,
									hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
									hash_table,
									collision_detected);
						}
					}
				}
			} else {
				//incrementCount == 3
				for (int iDirection = 0; iDirection < 3; iDirection++) {
					Vector3s potentiallyMissedBlockPosition = previousHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = currentHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound_Legacy_Algomorph(
								hash_entry_allocation_states,
								hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
								hash_table,
								collision_detected);
					}
					potentiallyMissedBlockPosition = currentHashBlockPosition;
					potentiallyMissedBlockPosition.values[iDirection] = previousHashBlockPosition.values[iDirection];
					if (SegmentIntersectsGridAlignedCube3D(segment_in_hash_blocks,
					                                       TO_FLOAT3(potentiallyMissedBlockPosition),
					                                       1.0f)) {
						MarkForAllocationAndSetVisibilityTypeIfNotFound_Legacy_Algomorph(
								hash_entry_allocation_states,
								hash_block_coordinates, hash_block_visibility_types, potentiallyMissedBlockPosition,
								hash_table,
								collision_detected);
					}
				}
			}
		}
		MarkForAllocationAndSetVisibilityTypeIfNotFound_Legacy_Algomorph(hash_entry_allocation_states,
		                                                                 hash_block_coordinates,
		                                                                 hash_block_visibility_types,
		                                                                 currentHashBlockPosition,
		                                                                 hash_table,
		                                                                 collision_detected);

		check_position += strideVector;
		previousHashBlockPosition = currentHashBlockPosition;
	}
}

_CPU_AND_GPU_CODE_
inline Vector3f cameraVoxelSpaceToWorldHashBlockSpaceLegacy_Algomorph(const Vector4f& point_camera_space_voxels,
                                                      const Matrix4f& inverted_camera_pose,
                                                      const float one_over_hash_block_size) {
	// account for the fact that voxel coordinates represent the voxel center, and we need the extreme corner position of
	// the hash block, i.e. 0.5 voxel (1/16 block) offset from the position along the ray
	return (TO_VECTOR3(inverted_camera_pose * point_camera_space_voxels)) * one_over_hash_block_size
	       + Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));
}

/**
 * \brief compute the segment formed by tracing the camera-space point a fixed distance forward and backward along
 * the ray; note: the starting & ending points of the returned segment are in voxel block coordinates
 * \param distance_from_point distance to trace backward and forward along the ray
 * \param point_in_camera_space point to trace from
 * \param inverted_camera_pose
 * \param one_over_hash_block_size
 * \return resulting segment in voxel block coordinates
 */
_CPU_AND_GPU_CODE_
inline ITMLib::Segment findHashBlockSegmentAlongCameraRayWithinRangeFromPointLegacy_Algomorph(
		const float distance_from_point,
		const Vector4f& point_in_camera_space,
		const Matrix4f& inverted_camera_pose,
		const float one_over_hash_block_size) {

	// distance to the point along camera ray
	float norm = ORUtils::length(point_in_camera_space);

	Vector4f endpoint_in_camera_space;

	endpoint_in_camera_space = point_in_camera_space * (1.0f - distance_from_point / norm);
	endpoint_in_camera_space.w = 1.0f;
	//start position along ray in hash blocks
	Vector3f start_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpaceLegacy_Algomorph(
			endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);

	endpoint_in_camera_space = point_in_camera_space * (1.0f + distance_from_point / norm);
	//end position of the segment to march along the ray
	Vector3f end_point_in_hash_blocks = cameraVoxelSpaceToWorldHashBlockSpaceLegacy_Algomorph(
			endpoint_in_camera_space, inverted_camera_pose, one_over_hash_block_size);

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment segment(start_point_in_hash_blocks, end_point_in_hash_blocks);
	return segment;
}

_CPU_AND_GPU_CODE_
inline Vector4f imageSpacePointToCameraSpaceLegacy_Algomorph(const float depth, const int x, const int y,
                                                             const Vector4f& inverted_camera_projection_parameters) {
	return {depth *
	        ((float(x) - inverted_camera_projection_parameters.z) * inverted_camera_projection_parameters.x),
	        depth *
	        ((float(y) - inverted_camera_projection_parameters.w) * inverted_camera_projection_parameters.y),
	        depth, 1.0f};
}

_CPU_AND_GPU_CODE_
inline ITMLib::Segment
findHashBlockSegmentAlongCameraRayWithinRangeFromDepthLegacy_Algomorph(const float distance_from_point,
                                                                       const float depth_measure,
                                                                       const int x, const int y,
                                                                       const Matrix4f& inverted_camera_pose,
                                                                       const Vector4f& inverted_camera_projection_parameters,
                                                                       const float one_over_hash_block_size) {

	Vector4f point_in_camera_space = imageSpacePointToCameraSpaceLegacy_Algomorph(depth_measure, x, y,
	                                                                              inverted_camera_projection_parameters);
	return findHashBlockSegmentAlongCameraRayWithinRangeFromPointLegacy_Algomorph(distance_from_point,
	                                                                              point_in_camera_space,
	                                                                              inverted_camera_pose,
	                                                                              one_over_hash_block_size);
}

_CPU_AND_GPU_CODE_ inline void
findVoxelBlocksForRayNearSurfaceLegacy_Algomorph(
		ITMLib::HashEntryAllocationState* hash_entry_allocation_states,
		Vector3s* hash_block_coordinates,
		ITMLib::HashBlockVisibility* hash_block_visibility_types,
		const CONSTPTR(HashEntry)* hash_table,
		const int x, const int y, const CONSTPTR(float)* depth, float surface_distance_cutoff,
		const Matrix4f inverted_camera_pose,
		const Vector4f inverted_projection_parameters,
		const float one_over_hash_block_size, const Vector2i depth_image_size,
		const float near_clipping_distance, const float far_clipping_distance,
		bool& collision_detected) {

	float depth_measure = depth[x + y * depth_image_size.x];
	if (depth_measure <= 0 || (depth_measure - surface_distance_cutoff) < 0 ||
	    (depth_measure - surface_distance_cutoff) < near_clipping_distance ||
	    (depth_measure + surface_distance_cutoff) > far_clipping_distance)
		return;

	// segment from start of the (truncated SDF) band, through the observed point, and to the opposite (occluded)
	// end of the (truncated SDF) band (increased by backBandFactor), along the ray cast from the camera through the
	// point, in camera space
	ITMLib::Segment march_segment = findHashBlockSegmentAlongCameraRayWithinRangeFromDepthLegacy_Algomorph(
			surface_distance_cutoff,
			depth_measure,
			x, y, inverted_camera_pose,
			inverted_projection_parameters,
			one_over_hash_block_size);


	findVoxelHashBlocksAlongSegmentLegacy_Algomorph(hash_entry_allocation_states, hash_block_coordinates,
	                                                hash_block_visibility_types,
	                                                hash_table, march_segment, collision_detected);
}


/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_d,
		const CONSTPTR(Vector4f)& projParams_d,
		float mu, int maxW,
		const CONSTPTR(float)* depth,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image (voxel point in camera coordinates)
	pt_camera = M_d * pt_model;
	// if point is behind the camera, don't modify any voxels
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	depth_measure = depth[(int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0f) return -1;

	// check whether voxel needs updating
	// eta = [depth at pixel corresp. to current ray] - [depth of voxel point along the ray]
	// effectively, eta is the distance between measured surface & voxel point
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf);
	oldW = voxel.w_depth;

	newF = ORUTILS_MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	// write back
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param confidence
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_d,
		const CONSTPTR(Vector4f)& projParams_d,
		float mu, int maxW,
		const CONSTPTR(float)* depth,
		const CONSTPTR(float)* confidence,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW, locId;

	// project point into image
	pt_camera = M_d * pt_model;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	locId = (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;
	// get measured depth from image
	depth_measure = depth[locId];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf);
	oldW = voxel.w_depth;
	newF = ORUTILS_MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	// write back^
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;
	voxel.confidence += TVoxel::floatToValue(confidence[locId]);

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_rgb,
		const CONSTPTR(Vector4f)& projParams_rgb,
		float mu, uchar maxW, float eta,
		const CONSTPTR(Vector4u)* rgb,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC;
	Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float) voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	voxel.clr = TO_UCHAR3(newC * 255.0f);
	voxel.w_color = (uchar) newW;
}

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, class TVoxel>
struct ComputeUpdatedVoxelInfo;

//================= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
float mu, int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined either as a constant or a preprocessor define -Greg (GitHub:Algomorph)
#define COMPUTE_COLOR_CHECK if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;

// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
//================= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================
// no color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};

#undef COMPUTE_COLOR_CHECK
#undef FLAG_UPDATE_CHECK
#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
//======================================================================================================================

