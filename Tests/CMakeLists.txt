#####################################
# CMakeLists.txt for Tests
#####################################

option(BUILD_TESTS "Build automated tests with the project (Boost-test required)." OFF)

set(COMMON_TEST_SOURCES
    TestUtilities.cpp
    TestUtilities.cu
    TestUtilities.tpp
    TestUtils_Kernels.h
    TestUtils_Functors.h
    SnoopyTestUtilities.cpp
    )
set(COMMON_TEST_HEADERS
    TestUtilities.h
    SnoopyTestUtilities.h)

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

    find_package(Boost COMPONENTS unit_test_framework REQUIRED)

    set(targetname test_utilities)
    set(sources ${COMMON_TEST_SOURCES} ${COMMON_TEST_HEADERS})

    load_use_includes()
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

    target_link_libraries(${targetname} ITMLib ORUtils
                          ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)

    #######################################################
    # generator executable for derived test data library  #
    #######################################################

    load_use_includes()
    set(targetname generate_derived_test_data)
    set(GENERATED_TEST_DATA_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
    configure_file(GenerateDerivedTestData_Config.h.in GenerateDerivedTestData_Config.h @ONLY)
    set(sources "GenerateDerivedTestData.cpp" "GenerateDerivedTestData_Config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/GenerateDerivedTestData_Config.h")
    include(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)
    target_link_libraries(${targetname} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib test_utilities)
    target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
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
    itm_add_test(NAME SceneSlicingPVA_CPU SOURCES Test_VoxelVolumeSlicingPVA_CPU.cpp)
    itm_add_test(NAME SurfaceTrackingAuxiliaryFunctions SOURCES Test_SurfaceTrackingAuxiliaryFunctions.cpp)
    itm_add_test(NAME WarpVolume SOURCES Test_WarpVolume.cpp)
    itm_add_test(NAME FuseLifeIntoCanonical SOURCES Test_FuseLifeIntoCanonical.cpp)
    itm_add_test(NAME WarpGradient_PVA_to_VBH SOURCES Test_WarpGradient_PVA_to_VBH.cpp
                 WarpAdvancedTestingUtilities.h WarpAdvancedTestingUtilities.tpp WarpAdvancedTestingUtilities.cpp)
    itm_add_test(NAME Configuration SOURCES Test_Configuration.cpp)
    itm_add_test(NAME GeometryUtilities SOURCES Test_GeometryUtilities.cpp)
    itm_add_test(NAME HashAllocationThreadSafety SOURCES Test_HashAllocationThreadSafety.cpp)
    itm_add_test(NAME TwoSurfaceHashAllocation SOURCES Test_TwoSurfaceHashAllocation.cpp)
    # itm_add_test(NAME ExpandedAllocation SOURCES Test_ExpandAllocation.cpp)
    itm_add_test(NAME Serialization SOURCES Test_Serialization.cpp)

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