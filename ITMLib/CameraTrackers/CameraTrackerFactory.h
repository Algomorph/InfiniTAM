// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>
#include <vector>

#include "CPU/ColorTracker_CPU.h"
#include "CPU/DepthTracker_CPU.h"
#include "CPU/ExtendedTracker_CPU.h"
#include "Interface/CompositeTracker.h"
#include "Interface/IMUTracker.h"
#include "Interface/FileBasedTracker.h"
#include "Interface/ForceFailTracker.h"
#include "Interface/CameraTracker.h"
#include "../Engines/LowLevel/Interface/LowLevelEngine.h"
#include "../Utils/Configuration.h"

#ifndef COMPILE_WITHOUT_CUDA

#include "CUDA/ColorTracker_CUDA.h"
#include "CUDA/DepthTracker_CUDA.h"
#include "CUDA/ExtendedTracker_CUDA.h"

#endif

#ifdef COMPILE_WITH_METAL
#include "Metal/ITMExtendedTracker_Metal.h"
#endif

#include "../../ORUtils/KeyValueConfig.h"
#include "CPU/SDF2SDFCameraTracker_CPU.h"
#include "CUDA/DynamicCameraTracker_CUDA.h"

namespace ITMLib {
/**
 * \brief An instance of this class can be used to construct trackers.
 */
class CameraTrackerFactory {
private:
	//#################### TYPEDEFS ####################
	typedef CameraTracker* MakerFunc(const Vector2i&, const Vector2i&, MemoryDeviceType,
	                                 const ORUtils::KeyValueConfig&, const LowLevelEngine*, IMUCalibrator*,
	                                 const VoxelVolumeParameters*);

	/// Tracker types
	typedef enum {
		//! Identifies a tracker based on colour image
				TRACKER_COLOR,
		//! Identifies a tracker based on depth image
				TRACKER_ICP,
		//! Identifies a tracker based on depth and color image with various extensions
				TRACKER_EXTENDED,
		//! Identifies a tracker reading poses from text files
				TRACKER_FILE,
		//! Identifies a tracker based on depth image and IMU measurement
				TRACKER_IMU,
		//! Identifies a tracker based on depth and colour images and IMU measurement
				TRACKER_EXTENDEDIMU,
		//! Identifies a tracker that forces tracking to fail
				TRACKER_FORCEFAIL,
		//! Identifies a tracker based on depth and color images that tracks dynamic scenes using a softened Killing constraint
				TRACKER_KILLING
	} TrackerType;

	struct Maker {
		const char* id;
		const char* description;
		TrackerType type;
		MakerFunc* make;

		Maker(const char* _id, const char* _desc, TrackerType _type, MakerFunc* _make)
				: id(_id), description(_desc), type(_type), make(_make) {}
	};

	//#################### PRIVATE VARIABLES ####################
	/** A list of maker functions for the various tracker types. */
	std::vector<Maker> makers;

	//################## SINGLETON IMPLEMENTATION ##################
	/**
	 * \brief Constructs a tracker factory.
	 */
	CameraTrackerFactory() {
		makers.push_back(Maker("rgb", "Colour based tracker", TRACKER_COLOR, &MakeColourTracker));
		makers.push_back(Maker("icp", "Depth based ICP tracker", TRACKER_ICP, &MakeICPTracker));
		makers.push_back(Maker("extended", "Depth + colour based tracker", TRACKER_EXTENDED, &MakeExtendedTracker));
		makers.push_back(Maker("file", "File based tracker", TRACKER_FILE, &MakeFileBasedTracker));
		makers.push_back(Maker("imuicp", "Combined IMU and depth based ICP tracker", TRACKER_IMU, &MakeIMUTracker));
		makers.push_back(Maker("extendedimu", "Combined IMU and depth + colour ICP tracker", TRACKER_EXTENDEDIMU,
		                       &MakeExtendedIMUTracker));
		makers.push_back(Maker("forcefail", "Force fail tracker", TRACKER_FORCEFAIL, &MakeForceFailTracker));
		makers.push_back(Maker("dynamic", "Dynamic scene camera tracker", TRACKER_KILLING,
		                       &MakeKillingTracker));
	}

public:
	/**
	 * \brief Gets the singleton instance for the current set of template parameters.
	 */
	static CameraTrackerFactory& Instance() {
		static CameraTrackerFactory s_instance;
		return s_instance;
	}

	//################## PUBLIC MEMBER FUNCTIONS ##################
public:
	/**
	 * \brief Makes a tracker of the type specified in the trackerConfig string.
	 */
	CameraTracker* Make(MemoryDeviceType deviceType, const char* trackerConfig, const Vector2i& imgSize_rgb,
	                    const Vector2i& imgSize_d, const LowLevelEngine* lowLevelEngine,
	                    IMUCalibrator* imuCalibrator, const VoxelVolumeParameters* sceneParams) const {
		ORUtils::KeyValueConfig cfg(trackerConfig);
		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;

		ORUtils::KeyValueConfig::ChoiceList trackerOptions;
		for (int i = 0; (unsigned) i < makers.size(); ++i) {
			trackerOptions.addChoice(makers[i].id, makers[i].type);
		}
		int type = TRACKER_ICP;
		cfg.parseChoiceProperty("type", "type of tracker", type, trackerOptions, verbose);
		const Maker* maker = NULL;
		for (int i = 0; (unsigned) i < makers.size(); ++i) {
			if (makers[i].type == type) {
				maker = &(makers[i]);
				break;
			}
		}
		if (maker == NULL) DIEWITHEXCEPTION("Unknown tracker type");

		CameraTracker* ret = (*(maker->make))(imgSize_rgb, imgSize_d, deviceType, cfg, lowLevelEngine, imuCalibrator,
		                                      sceneParams);
		if (ret->requiresColourRendering()) {
			printf("Assuming a voxel type with colour information!");
		}

		return ret;
	}

	/**
	 * \brief Makes a tracker of the type specified in the settings.
	 */
	CameraTracker* Make(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, const LowLevelEngine* lowLevelEngine,
	                    IMUCalibrator* imuCalibrator, const VoxelVolumeParameters* sceneParams) const {
		auto& settings = configuration::get();
		return Make(settings.device_type, settings.tracker_configuration.c_str(), imgSize_rgb, imgSize_d, lowLevelEngine,
		            imuCalibrator, sceneParams);
	}

	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################
	static std::vector<TrackerIterationType> parseLevelConfig(const char* str) {
		bool parseError = false;
		std::vector<TrackerIterationType> ret;
		for (int i = static_cast<int>(strlen(str)) - 1; i >= 0; --i) {
			switch (str[i]) {
				case 'r':
					ret.push_back(TRACKER_ITERATION_ROTATION);
					break;
				case 't':
					ret.push_back(TRACKER_ITERATION_TRANSLATION);
					break;
				case 'b':
					ret.push_back(TRACKER_ITERATION_BOTH);
					break;
				case 'n':
					ret.push_back(TRACKER_ITERATION_NONE);
					break;
				default:
					parseError = true;
					break;
			}
		}

		if (parseError) {
			fprintf(stderr, "error parsing level configuration '%s'\n", str);
			for (int i = 0; (unsigned) i < ret.size(); ++i)
				fprintf(stderr, "level %i: %i\n", (int) ret.size() - i, (int) (ret[ret.size() - i]));
		}
		return ret;
	}

	/**
	 * \brief Makes a colour tracker.
	 */
	static CameraTracker*
	MakeColourTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	                  const ORUtils::KeyValueConfig& cfg,
	                  const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                  const VoxelVolumeParameters* sceneParams) {
		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;

		const char* levelSetup = "rrrbb";
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		ColorTracker* ret = NULL;
		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				ret = new ColorTracker_CPU(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()),
				                           lowLevelEngine);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				ret = new ColorTracker_CUDA(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()),
				                               lowLevelEngine);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				ret = new ColorTracker_CPU(imgSize_rgb, &(levels[0]), static_cast<int>(levels.size()), lowLevelEngine);
#endif
				break;
		}

		if (ret == NULL) DIEWITHEXCEPTION("Failed to make colour tracker");
		return ret;
	}

	/**
	 * \brief Makes an ICP tracker.
	 */
	static CameraTracker*
	MakeICPTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	               const ORUtils::KeyValueConfig& cfg,
	               const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	               const VoxelVolumeParameters* sceneParams) {
		const char* levelSetup = "rrrbb";
		float smallStepSizeCriterion = 1e-3f;
		float outlierDistanceFine = 0.002f;
		float outlierDistanceCoarse = 0.01f;
		float failureDetectorThd = 3.0f;
		int numIterationsCoarse = 10;
		int numIterationsFine = 2;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierC", "outlier threshold at coarsest level", outlierDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierF", "outlier threshold at finest level", outlierDistanceFine, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse,
		                     verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		DepthTracker* ret = NULL;
		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				ret = new DepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()),
				                           smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				ret = new DepthTracker_CUDA(imgSize_d, &(levels[0]), static_cast<int>(levels.size()),
				                               smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				ret = new DepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
				break;
		}

		if (ret == NULL) DIEWITHEXCEPTION("Failed to make ICP tracker");
		ret->SetupLevels(numIterationsCoarse, numIterationsFine,
		                 outlierDistanceCoarse, outlierDistanceFine);
		return ret;
	}

	template<
			typename TTracker_CPU
#ifndef COMPILE_WITHOUT_CUDA
			, typename TTracker_CUDA
#endif
#ifdef COMPILE_WITH_METAL
			,typename TTracker_METAL
#endif
	>
	static CameraTracker* MakeExtendedLikeTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d,
	                                              MemoryDeviceType deviceType,
	                                              const ORUtils::KeyValueConfig& cfg,
	                                              const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                                              const VoxelVolumeParameters* sceneParams) {

		//ensure template arguments derive from correct types
		static_assert(std::is_base_of<ExtendedTracker_CPU, TTracker_CPU>::value,
		              "TTracker_CPU must be derived from ITMExtendedTracker_CPU");
#ifndef COMPILE_WITHOUT_CUDA
		static_assert(std::is_base_of<ExtendedTracker_CUDA, TTracker_CUDA>::value,
		              "TTracker_CUDA must be derived from ITMExtendedTracker_CUDA");
#endif
#ifdef COMPILE_WITH_METAL
		static_assert(std::is_base_of<ITMExtendedTracker_Metal, TTracker_METAL>::value, "TTracker_CUDA must be derived from ITMExtendedTracker_CUDA");
#endif
		const char* levelSetup = "rrbb";
		bool useDepth = true;
		bool useColour = false;
		float colourWeight = 0.3f;
		float smallStepSizeCriterion = 1e-4f;
		float outlierSpaceDistanceFine = 0.004f;
		float outlierSpaceDistanceCoarse = 0.1f;
		float outlierColourDistanceFine = 0.175f;
		float outlierColourDistanceCoarse = 0.005f;
		float failureDetectorThd = 3.0f;
		float minColourGradient = 0.01f;
		float tukeyCutOff = 8.0f;
		int framesToSkip = 20;
		int framesToWeight = 50;
		int numIterationsCoarse = 20;
		int numIterationsFine = 20;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseBoolProperty("useDepth", "use ICP based tracking", useDepth, verbose);
		cfg.parseBoolProperty("useColour", "use colour based tracking", useColour, verbose);
		cfg.parseFltProperty("colourWeight",
		                     "weight used to scale colour errors and jacobians when both useColour and useWeights are set",
		                     colourWeight, verbose);
		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierSpaceC", "space outlier threshold at coarsest level", outlierSpaceDistanceCoarse,
		                     verbose);
		cfg.parseFltProperty("outlierSpaceF", "space outlier threshold at finest level", outlierSpaceDistanceFine,
		                     verbose);
		cfg.parseFltProperty("outlierColourC", "colour outlier threshold at coarsest level",
		                     outlierColourDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierColourF", "colour outlier threshold at finest level", outlierColourDistanceFine,
		                     verbose);
		cfg.parseFltProperty("minColourGradient", "minimum colour gradient for a pixel to be used in the tracking",
		                     minColourGradient, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse,
		                     verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("tukeyCutOff", "cutoff for the tukey m-estimator", tukeyCutOff, verbose);
		cfg.parseIntProperty("framesToSkip", "number of frames to skip before depth pixel is used for tracking",
		                     framesToSkip, verbose);
		cfg.parseIntProperty("framesToWeight", "number of frames to weight each depth pixel for before using it fully",
		                     framesToWeight, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		ExtendedTracker* ret = NULL;
		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				ret = new TTracker_CPU(imgSize_d,
				                       imgSize_rgb,
				                       useDepth,
				                       useColour,
				                       colourWeight,
				                       &(levels[0]),
				                       static_cast<int>(levels.size()),
				                       smallStepSizeCriterion,
				                       failureDetectorThd,
				                       sceneParams->near_clipping_distance,
				                       sceneParams->far_clipping_distance,
				                       minColourGradient,
				                       tukeyCutOff,
				                       framesToSkip,
				                       framesToWeight,
				                       lowLevelEngine);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				ret = new TTracker_CUDA(imgSize_d,
				                        imgSize_rgb,
				                        useDepth,
				                        useColour,
				                        colourWeight,
				                        &(levels[0]),
				                        static_cast<int>(levels.size()),
				                        smallStepSizeCriterion,
				                        failureDetectorThd,
				                        sceneParams->near_clipping_distance,
				                        sceneParams->far_clipping_distance,
				                        minColourGradient,
				                        tukeyCutOff,
				                        framesToSkip,
				                        framesToWeight,
				                        lowLevelEngine);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				ret = new TTracker_METAL(imgSize_d, imgSize_rgb, useDepth, useColour, colourWeight, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd,
				volume->voxel_volume_parameters->near_clipping_distance, volume->voxel_volume_parameters->far_clipping_distance, tukeyCutOff, framesToSkip, framesToWeight, lowLevelEngine);
#endif
				break;
		}

		if (ret == NULL) DIEWITHEXCEPTION("Failed to make extended tracker");
		ret->SetupLevels(numIterationsCoarse, numIterationsFine, outlierSpaceDistanceCoarse, outlierSpaceDistanceFine,
		                 outlierColourDistanceCoarse, outlierColourDistanceFine);
		return ret;
	}

	/**
	* \brief Makes an Extended tracker.
	*/
	static CameraTracker*
	MakeExtendedTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	                    const ORUtils::KeyValueConfig& cfg,
	                    const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                    const VoxelVolumeParameters* sceneParams) {
		return
				MakeExtendedLikeTracker<
						ExtendedTracker_CPU
#ifndef COMPILE_WITHOUT_CUDA
						, ExtendedTracker_CUDA
#endif
#ifdef COMPILE_WITH_METAL
						, ExtendedTracker_Metal
#endif
				>(imgSize_rgb, imgSize_d, deviceType, cfg, lowLevelEngine, imuCalibrator, sceneParams);


	}

	/**
	 * \brief Makes a KillingFusion dynamic-scene tracker.
	 */
	static CameraTracker* MakeKillingTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d,
	                                         MemoryDeviceType deviceType,
	                                         const ORUtils::KeyValueConfig& cfg,
	                                         const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                                         const VoxelVolumeParameters* sceneParams) {
		return MakeExtendedLikeTracker<
				SDF2SDFCameraTracker_CPU
#ifndef COMPILE_WITHOUT_CUDA
				, DynamicCameraTracker_CUDA
#endif
#ifdef COMPILE_WITH_METAL
				, KillingTracker_Metal //TODO
#endif
		>(imgSize_rgb, imgSize_d, deviceType, cfg, lowLevelEngine, imuCalibrator, sceneParams);
	}

	/**
	 * \brief Makes an IMU tracker.
	 */
	static CameraTracker*
	MakeIMUTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	               const ORUtils::KeyValueConfig& cfg,
	               const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	               const VoxelVolumeParameters* sceneParams) {
		const char* levelSetup = "tb";
		float smallStepSizeCriterion = 1e-3f;
		float outlierDistanceFine = 0.005f;
		float outlierDistanceCoarse = 0.01f;
		float failureDetectorThd = 3.0f;
		int numIterationsCoarse = 4;
		int numIterationsFine = 2;

		int verbose = 0;
		if (cfg.getProperty("help") != NULL) if (verbose < 10) verbose = 10;
		cfg.parseStrProperty("levels", "resolution hierarchy levels", levelSetup, verbose);
		std::vector<TrackerIterationType> levels = parseLevelConfig(levelSetup);

		cfg.parseFltProperty("minstep", "step size threshold for convergence", smallStepSizeCriterion, verbose);
		cfg.parseFltProperty("outlierC", "outlier threshold at coarsest level", outlierDistanceCoarse, verbose);
		cfg.parseFltProperty("outlierF", "outlier threshold at finest level", outlierDistanceFine, verbose);
		cfg.parseIntProperty("numiterC", "maximum number of iterations at coarsest level", numIterationsCoarse,
		                     verbose);
		cfg.parseIntProperty("numiterF", "maximum number of iterations at finest level", numIterationsFine, verbose);
		cfg.parseFltProperty("failureDec", "threshold for the failure detection", failureDetectorThd, verbose);

		DepthTracker* dTracker = NULL;
		switch (deviceType) {
			case MEMORYDEVICE_CPU:
				dTracker = new DepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()),
				                                smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				dTracker = new DepthTracker_CUDA(imgSize_d, &(levels[0]), static_cast<int>(levels.size()),
				                                    smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				dTracker = new DepthTracker_CPU(imgSize_d, &(levels[0]), static_cast<int>(levels.size()), smallStepSizeCriterion, failureDetectorThd, lowLevelEngine);
#endif
				break;
			default:
				break;
		}

		if (dTracker == NULL) DIEWITHEXCEPTION("Failed to make IMU tracker");
		dTracker->SetupLevels(numIterationsCoarse, numIterationsFine,
		                      outlierDistanceCoarse, outlierDistanceFine);

		CompositeTracker* compositeTracker = new CompositeTracker;
		compositeTracker->AddTracker(new IMUTracker(imuCalibrator));
		compositeTracker->AddTracker(dTracker);
		return compositeTracker;
	}

	/**
	* \brief Makes an Extended IMU tracker.
	*/
	static CameraTracker* MakeExtendedIMUTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d,
	                                             MemoryDeviceType deviceType, const ORUtils::KeyValueConfig& cfg,
	                                             const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                                             const VoxelVolumeParameters* sceneParams) {
		CameraTracker* dTracker = MakeExtendedTracker(imgSize_rgb, imgSize_d, deviceType, cfg,
		                                              lowLevelEngine, imuCalibrator, sceneParams);
		if (dTracker == NULL) DIEWITHEXCEPTION("Failed to make extended tracker"); // Should never happen though

		CompositeTracker* compositeTracker = new CompositeTracker;
		compositeTracker->AddTracker(new IMUTracker(imuCalibrator));
		compositeTracker->AddTracker(dTracker);
		return compositeTracker;
	}

	/**
	 * \brief Makes a file based tracker.
	 */
	static CameraTracker*
	MakeFileBasedTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	                     const ORUtils::KeyValueConfig& cfg,
	                     const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                     const VoxelVolumeParameters* sceneParams) {
		int verbose = 0;
		if (cfg.getProperty("help") && verbose < 10) verbose = 10;

		const char* fileMask = "";
		cfg.parseStrProperty("mask", "mask for the saved pose text files", fileMask, verbose);

		return new FileBasedTracker(fileMask);
	}

	/**
	 * \brief Makes a force fail tracker.
	 */
	static CameraTracker*
	MakeForceFailTracker(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d, MemoryDeviceType deviceType,
	                     const ORUtils::KeyValueConfig& cfg,
	                     const LowLevelEngine* lowLevelEngine, IMUCalibrator* imuCalibrator,
	                     const VoxelVolumeParameters* sceneParams) {
		return new ForceFailTracker;
	}

};
} //namepace ITMLib
