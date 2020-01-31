// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"
#include "../../Utils/ITMVoxelFlags.h"

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_f_rgb
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %f, w_depth: %d, clr: [%d, %d, %d], w_color: %d}\n",
				sdf, w_depth, clr.r, clr.g, clr.b, w_color);
	}
};

/** \brief
    Stores the information of a single voxel in the volume
*/
struct ITMVoxel_s_rgb
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %d, w_depth: %d,clr::[%d, %d, %d],w_color: %d}\n",
		       sdf, w_depth, clr.r, clr.g, clr.b, w_color);
	}
};

struct ITMVoxel_s
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_s()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf:%d,w_depth:%d}\n", sdf, w_depth);
	}
};

struct ITMVoxel_f
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;

	_CPU_AND_GPU_CODE_ ITMVoxel_f()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %f, w_depth: %d}\n", sdf, w_depth);
	}
};

struct ITMVoxel_f_conf
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %f, w_depth: %d, conf: %f}\n",
		       sdf, w_depth, confidence);
	}
};


struct ITMVoxel_s_rgb_conf
{
	_CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float)(x) / 32767.0f; }
	_CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short)((x) * 32767.0f); }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	short sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_s_rgb_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %d, w_depth: %d, clr: [%d, %d, %d], w_color: %d, confidence: %f}\n",
		       sdf, w_depth, clr.r, clr.g, clr.b, w_color, confidence);
	}
};

struct ITMVoxel_f_rgb_conf
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = true;
	static const CONSTPTR(bool) hasConfidenceInformation = true;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** Padding that may or may not improve performance on certain GPUs */
	//uchar pad;
	/** RGB colour information stored for this voxel. */
	Vector3u clr;
	/** Number of observations that made up @p clr. */
	uchar w_color;
	float confidence;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_rgb_conf()
	{
		sdf = SDF_initialValue();
		w_depth = 0;
		confidence = 0.0f;
		clr = Vector3u((uchar)0);
		w_color = 0;
	}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %f, w_depth: %d, clr: [%d, %d, %d], w_color: %d, confidence: %f}\n",
		       sdf, w_depth, clr.r, clr.g, clr.b, w_color, confidence);
	}
};


struct ITMVoxel_f_warp{
	static const CONSTPTR(bool) hasSDFInformation = false;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = false;
	static const CONSTPTR(bool) hasWeightInformation = false;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasFramewiseWarp = true;
	static const CONSTPTR(bool) hasWarpUpdate = true;
	static const CONSTPTR(bool) hasDebugInformation = false;
	/** vector translating the current point to a different location **/
	Vector3f framewise_warp;
	/** intermediate results for computing the gradient & the points motion**/
	union{
		Vector3f gradient0;
		Vector3f warp_update;
		Vector3f gradient;
	};
	Vector3f gradient1;

	_CPU_AND_GPU_CODE_ ITMVoxel_f_warp():
		framewise_warp(0.0f),
		gradient0(0.0f),
		gradient1(0.0f)
		{}
	_CPU_AND_GPU_CODE_ void print_self(){
		printf("warp:{framewise_warp: [%f, %f, %f], gradient0: [%f, %f, %f], gradient1: [%f, %f, %f]}\n",
		       framewise_warp.x, framewise_warp.y, framewise_warp.z, gradient0.x, gradient0.y, gradient0.z,
		       gradient1.x, gradient1.y, gradient1.z);
	}
};


struct ITMVoxel_f_flags
{
	_CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
	_CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
	_CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

	static const CONSTPTR(bool) hasSDFInformation = true;
	static const CONSTPTR(bool) hasColorInformation = false;
	static const CONSTPTR(bool) hasConfidenceInformation = false;
	static const CONSTPTR(bool) hasSemanticInformation = true;
	static const CONSTPTR(bool) hasWeightInformation = true;
	static const CONSTPTR(bool) hasCumulativeWarp = false;
	static const CONSTPTR(bool) hasFramewiseWarp = false;
	static const CONSTPTR(bool) hasWarpUpdate = false;
	static const CONSTPTR(bool) hasDebugInformation = false;

	/** Value of the truncated signed distance transformation. */
	float sdf;
	/** Number of fused observations that make up @p sdf. */
	uchar w_depth;
	/** refer to ITMVoxelFlags for flag bit array values */
	unsigned char flags;
	_CPU_AND_GPU_CODE_ ITMVoxel_f_flags() :
			flags(ITMLib::VOXEL_UNKNOWN),
			sdf(SDF_initialValue()),
			w_depth(0){}

	_CPU_AND_GPU_CODE_ void print_self(){
		printf("voxel:{sdf: %f, w_depth: %d, flags: %d}\n",
		       sdf, w_depth, flags);
	}
};
