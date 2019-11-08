// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cfloat>
#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "ITMMath.h"
#include "../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"

//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace ITMLib
{
	class Configuration
	{
	public:
		static void SetFromVariableMap(const po::variables_map& vm);

		static Configuration& Instance(){
			static Configuration instance;
			return instance;
		}

		~Configuration() = default;

		// Suppress the default copy constructor and assignment operator
		Configuration(const Configuration &) = delete;
		Configuration& operator=(const Configuration &) = delete;


		// accepting program arguments from boost::program_options. One of the arguments may be a .json config file, which should
		// then be read in using boost::property_tree. The program_options parser should also be generated within this
		// class. -Greg (GitHub: Algomorph)
		//TODO: settings that are not intended to be changed during runtime should be set to const and initialized
		// right away in the constructor. Settings that are intended to change should be protected by class access
		// modifiers / getters / setters as to grant access only to the objects that should be able to change them.
		// -Greg (GitHub: Algomorph)

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;
        
		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE,
			LIBMODE_DYNAMIC
		}LibMode;



		/// Select the type of device to use
		MemoryDeviceType deviceType;

		bool useApproximateRaycast;

		bool useThresholdFilter;
		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;
        
		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		const char *trackerConfig;

		/// Further, scene-specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		MemoryDeviceType GetMemoryType() const;

		/// Dynamic Fusion parameters
		struct AnalysisSettings{
			std::string outputPath;
			bool focusCoordinatesSpecified = false;
			Vector3i focusCoordinates;
		};

		bool restrictZtrackingForDebugging;// = false;
		bool FocusCoordinatesAreSpecified() const;// = false; // CLI flag made in InfiniTAM_bpo

		Vector3i GetFocusCoordinates() const;
		void SetFocusCoordinates(const Vector3i& coordiantes);
		void SetFocusCoordinates(int x, int y, int z);

		AnalysisSettings analysisSettings;

		//*** Scene Tracking ITMSceneMotionOptimizationSwitches ***

		bool enableDataTerm;
		bool enableLevelSetTerm;
		bool enableSmoothingTerm;
		bool enableKillingConstraintInSmoothingTerm;
		bool enableGradientSmoothing;

		//*** Scene Tracking ITMSceneMotionOptimizationParameters ***
		//** optimization loop
		unsigned int sceneTrackingMaxOptimizationIterationCount;
		float sceneTrackingOptimizationVectorUpdateThresholdMeters;
		//** gradient calculation
		float sceneTrackingGradientDescentLearningRate;
		float sceneTrackingRigidityEnforcementFactor;
		float sceneTrackingWeightDataTerm;
		float sceneTrackingWeightSmoothingTerm;
		float sceneTrackingWeightLevelSetTerm;
		float sceneTrackingLevelSetTermEpsilon;

	private:
		Configuration();
	};
}