#####################################
# CMakeLists.txt for Tests
#####################################

option(BUILD_TESTS "Build automated tests with the project (Boost-test required)." OFF)



macro(load_use_includes)
    include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseFFmpeg.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)
endmacro()

macro(load_target_link_includes)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
endmacro()

macro(itm_add_test)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(ITM_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    set(targetname test_${ITM_ADD_TEST_NAME})
    set(sources ${ITM_ADD_TEST_SOURCES})

    load_use_includes()

    ##########################################
    # Specify the target and where to put it #
    ##########################################
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

    target_link_libraries(${targetname}
                          InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib test_utilities
                          ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                          )

    load_target_link_includes()

    ##########################################
    # Add the target as a CMake CTest        #
    ##########################################
    add_test(NAME ${ITM_ADD_TEST_NAME} COMMAND test_${ITM_ADD_TEST_NAME})
endmacro()

if (BUILD_TESTS)
    enable_testing()

    ###########################
    # test_utilities library  #
    ###########################
    set(GENERATED_TEST_DATA_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
    configure_file(TestUtilities/TestUtilitiesConfig.h.in TestUtilities/TestUtilitiesConfig.h @ONLY)

    set(TEST_UTILITIES_SOURCES
        TestUtilities/TestUtilities.cpp
        TestUtilities/TestUtilities.cu
        TestUtilities/TestUtilities.tpp
        TestUtilities/SnoopyTestUtilities.cpp
        TestUtilities/WarpAdvancedTestingUtilities.tpp
        TestUtilities/WarpAdvancedTestingUtilities.cpp
        )
    set(TEST_UTILITIES_HEADERS
        TestUtilities/TestUtilityKernels.h
        TestUtilities/TestUtilityFunctors.h
        TestUtilities/TestUtilities.h
        TestUtilities/SnoopyTestUtilities.h
        TestUtilities/WarpAdvancedTestingUtilities.h
        TestUtilities/TestUtilitiesConfig.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/TestUtilities/TestUtilitiesConfig.h)
    
    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    set(targetname test_utilities)
    set(sources ${TEST_UTILITIES_SOURCES} ${TEST_UTILITIES_HEADERS})

    load_use_includes()
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

    target_link_libraries(${targetname} ITMLib ORUtils
                          ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/TestUtilities)
    include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)

    #######################################################
    # generator executable for derived test data library  #
    #######################################################
    set(targetname generate_derived_test_data)
    load_use_includes()
    set(sources GenerateDerivedTestData.cpp)
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)
    target_link_libraries(${targetname} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib test_utilities)
    load_target_link_includes()

    ##########################
    # test suite executables #
    ##########################

    # *** CPU-only tests
    itm_add_test(NAME SetCopyCompare_CPU SOURCES Test_SetCopyCompare_CPU.cpp)
    itm_add_test(NAME ConstructVolumeFromImage_CPU SOURCES Test_ConstructVolumeFromImage_CPU.cpp)
    itm_add_test(NAME VolumeSaveLoadCompact_CPU SOURCES Test_VolumeSaveLoadCompact_CPU.cpp)
    itm_add_test(NAME IntArrayMap3D SOURCES Test_IntArrayMap3D.cpp)
    itm_add_test(NAME ImageMaskReader SOURCES Test_ImageMaskReader.cpp)
    itm_add_test(NAME WarpGradient_CPU_PVA SOURCES Test_WarpGradient_CPU_PVA.cpp Test_WarpGradient_Common.h)
    itm_add_test(NAME WarpGradient_CPU_VBH SOURCES Test_WarpGradient_CPU_VBH.cpp Test_WarpGradient_Common.h)
    itm_add_test(NAME VolumeSlicingPVA_CPU SOURCES Test_VoxelVolumeSlicingPVA_CPU.cpp)
    itm_add_test(NAME SurfaceTrackingAuxiliaryFunctions SOURCES Test_SurfaceTrackingAuxiliaryFunctions.cpp)
    itm_add_test(NAME WarpVolume SOURCES Test_WarpVolume.cpp)
    itm_add_test(NAME FuseLifeIntoCanonical SOURCES Test_FuseLifeIntoCanonical.cpp)
    itm_add_test(NAME WarpGradient_PVA_to_VBH SOURCES Test_WarpGradient_PVA_to_VBH.cpp)
    itm_add_test(NAME Configuration SOURCES Test_Configuration.cpp)
    itm_add_test(NAME GeometryUtilities SOURCES Test_GeometryUtilities.cpp)
    itm_add_test(NAME HashAllocationThreadSafety SOURCES Test_HashAllocationThreadSafety.cpp)
    itm_add_test(NAME TwoSurfaceHashAllocation SOURCES Test_TwoSurfaceHashAllocation.cpp)
    itm_add_test(NAME MeshGeneration SOURCES Test_MeshGeneration.cpp)
    itm_add_test(NAME EnumSerialization SOURCES Test_EnumSerialization.cpp)

    # *** tests that require CUDA
    if(WITH_CUDA)
        itm_add_test(NAME SetCopyCompare_CUDA SOURCES Test_SetCopyCompare_CUDA.cpp)
        itm_add_test(NAME ConstructVolumeFromImage_CUDA SOURCES Test_ConstructVolumeFromImage_CUDA.cpp)
        itm_add_test(NAME ConstructVolumeFromImage_CPU_to_CUDA SOURCES
                     Test_ConstructVolumeFromImage_CPU_to_CUDA.cpp)
        itm_add_test(NAME VolumeSaveLoadCompact_CUDA SOURCES Test_VolumeSaveLoadCompact_CUDA.cpp)
        itm_add_test(NAME WarpGradient_CUDA_PVA SOURCES Test_WarpGradient_CUDA_PVA.cpp Test_WarpGradient_Common.h)
        itm_add_test(NAME WarpGradient_CUDA_VBH SOURCES Test_WarpGradient_CUDA_VBH.cpp Test_WarpGradient_Common.h)
        itm_add_test(NAME CUDA_Atomics SOURCES Test_CUDA_Atomics.cpp CUDAAtomicTesting.h
                     CUDAAtomicTesting.cu TemporaryCUDA_Atomics.h)
        itm_add_test(NAME SmoothWarpGradient SOURCES Test_SmoothWarpGradient_CPU_CUDA.cpp)
        itm_add_test(NAME VolumeReduction_VBH_CUDA SOURCES Test_VolumeReduction_VBH_CUDA.cpp)
    endif()

    #######################################################
    # Recursively transfer test data to build directories #
    #######################################################

    file(GLOB_RECURSE TEST_DATA_FILES
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.csv"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.png"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.pnm"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.txt"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.dat"
         "${CMAKE_CURRENT_SOURCE_DIR}/TestData/*.json"
         )
    set(TEST_DATA_DIRS)
    if(MSVC_IDE)
        foreach (CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
            file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CONFIGURATION_TYPE}")
            file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CONFIGURATION_TYPE}/TestData")
            list(APPEND TEST_DATA_DIRS "${CMAKE_CURRENT_BINARY_DIR}/${CONFIGURATION_TYPE}/TestData")
        endforeach (CONFIGURATION_TYPE)
    else()
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestData")
        list(APPEND TEST_DATA_DIRS "${CMAKE_CURRENT_BINARY_DIR}/TestData")
    endif()
    foreach (TEST_DATA_DIR ${TEST_DATA_DIRS})
        foreach (TEST_DATA_FILE ${TEST_DATA_FILES})
            string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR}/TestData ${TEST_DATA_DIR} NEW_FILE ${TEST_DATA_FILE})
            get_filename_component(TARGET_DIR ${NEW_FILE} DIRECTORY)
            file(MAKE_DIRECTORY ${TARGET_DIR})
            configure_file(
                    ${TEST_DATA_FILE}
                    ${NEW_FILE}
                    COPYONLY
            )
        endforeach (TEST_DATA_FILE)
    endforeach (TEST_DATA_DIR)
endif ()