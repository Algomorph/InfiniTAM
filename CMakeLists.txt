##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

project(InfiniTAM)

#################################
# CMake policies #
#################################
if (POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif ()
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif ()
if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

#################################
# Enforce C++ language standards
#################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)

#################################
# Specify the CMake module path #
#################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#################################
# Add additional compiler flags #
#################################

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
endif ()


if (NOT MSVC_IDE)
    set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
    set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_RELEASE}")
    set(CMAKE_C_FLAGS_DEBUG "-g -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_DEBUG}")
    #SET(CMAKE_CXX_FLAGS "-g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

##########################
# Some compiler fixes    #
##########################

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND ${CMAKE_CXX_STANDARD} STREQUAL "17" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0.1)
    # fix filesystem "bug" with GCC < 9.0.1 (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=90050)
    link_libraries(stdc++fs)
endif()

# Suppress auto-linking

if (MSVC_IDE)
    add_definitions(-DUSING_CMAKE=1)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/MP>)
endif ()


file(GLOB_RECURSE CMAKE_SOURCES "cmake/*")
add_custom_target(cmake SOURCES ${CMAKE_SOURCES})
source_group(CMake FILES ${CMAKE_SOURCES})

######################
# Add subdirectories #
######################

add_subdirectory(external)
add_subdirectory(Apps)
add_subdirectory(FernRelocLib)
add_subdirectory(InputSource)
add_subdirectory(ITMLib)
add_subdirectory(MiniSlamGraphLib)
add_subdirectory(ORUtils)
add_subdirectory(Tests)

