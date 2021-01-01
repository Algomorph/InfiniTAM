##########################################
# Top-level CMakeLists.txt               #
##########################################
cmake_minimum_required(VERSION 3.18.2 FATAL_ERROR)

# uncomment to time compile units
# set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

#################################
# CMake policies                #
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
if (POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
endif()

##################################
# Specify the CMake module paths #
##################################

list(APPEND CMAKE_MODULE_PATH 
    "${CMAKE_SOURCE_DIR}/cmake/"
    "${CMAKE_SOURCE_DIR}/cmake/debugging/"
    "${CMAKE_SOURCE_DIR}/cmake/dependency_setup/")

#################################
# Handle CUDA option            #
#################################

# Use the new CMake CUDA mechanism which enables full Nsight support
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER_ID)
    option(WITH_CUDA "Build with CUDA support?" ${CMAKE_CUDA_COMPILER})
endif()
# for all CMake versions (possibly revisions needed later)
if (NOT WITH_CUDA)
    add_definitions(-DCOMPILE_WITHOUT_CUDA)
else()
    enable_language(CUDA)
endif ()

if(WITH_CUDA)
    project(Reco LANGUAGES CXX C CUDA)
else()
    project(Reco LANGUAGES CXX C)
endif()

#################################
# Enforce language standards    #
#################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
if(WITH_CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
endif()


#################################
# Add additional compiler flags #
#################################

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
endif ()


if (NOT MSVC)
    set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
    set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_RELEASE}")
    set(CMAKE_C_FLAGS_DEBUG "-g -march=native ${CFLAGS_WARN} ${CMAKE_C_FLAGS_DEBUG}")
endif ()


# If on Mac OS X:
if (${CMAKE_SYSTEM} MATCHES "Darwin")
    # Make sure that C++11 warnings are disabled.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")

    # Make sure that the template depth is sufficient.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=512")

    if (${CMAKE_SYSTEM} MATCHES "Darwin-13.")
        # If on Mac OS X 10.9 (Mavericks), use the libstdc++ implementation of the C++ Standard Library and prevent C++11 code from being compiled.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
        add_definitions(-DNO_CPP11)
    else ()
        # Otherwise, use the libc++ implementation of the C++ Standard Library, and enable C++11 support.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -std=c++11")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -std=c++11")
    endif ()
endif ()

# If on Windows and using Visual Studio:
if (MSVC)
    # Disable the annoying warnings about using secure CRT functions (they're Microsoft-specific, so we can't use them portably).
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)

    # Prevent the definitions of min and max when including windows.h.
    add_definitions(-DNOMINMAX)

    # Make sure that the maths constants are defined.
    add_definitions(-D_USE_MATH_DEFINES)
endif ()

# CUDA-specific
if (WITH_CUDA)
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda")
    add_compile_options($<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CUDA_COMPILER_ID:NVIDIA>>:-Wno-deprecated-gpu-targets>)
endif ()

##########################
# Some compiler fixes    #
##########################

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND ${CMAKE_CXX_STANDARD} STREQUAL "17" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0.1)
    # fix filesystem "bug" with GCC < 9.0.1 (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=90050)
    link_libraries(stdc++fs)
endif()

# Suppress auto-linking
if (MSVC)
    add_definitions(-DUSING_CMAKE=1)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/MP>)
endif ()


file(GLOB_RECURSE CMAKE_SOURCES "cmake/*")
add_custom_target(cmake SOURCES ${CMAKE_SOURCES})
source_group(CMake FILES ${CMAKE_SOURCES})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

###########################
# Add system dependencies #
###########################
include(SystemDependencies)

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

