#################
# UseCUDA.cmake #
#################

# TODO: test that with MSVC, the desired effect is achieved with CMake v. <3.9 (CMake boolean/if syntax is kind-of obscure)
if (${CMAKE_VERSION} VERSION_LESS 3.8 OR (MSVC_IDE AND ${CMAKE_VERSION} VERSION_LESS 3.9))
    find_package(CUDA QUIET)
    option(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
else ()
    # Use the new CMake mechanism wich enables full Nsight support
    include(CheckLanguage)
    check_language(CUDA)
    option(WITH_CUDA "Build with CUDA support?" ${CMAKE_CUDA_COMPILER})
endif ()

# for all CMake versions (possibly revisions needed later)
if (WITH_CUDA)
    if (${CMAKE_VERSION} VERSION_LESS 3.8 OR (MSVC_IDE AND ${CMAKE_VERSION} VERSION_LESS 3.9))
        # Auto-detect the CUDA compute_allocated capability.
        set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
        if (NOT DEFINED CUDA_COMPUTE_CAPABILITY)
            include("${CMAKE_MODULE_PATH}/CUDACheckCompute.cmake")
        endif ()

        # Set the compute_allocated capability flags.
        foreach (compute_capability ${CUDA_COMPUTE_CAPABILITY})
            list(APPEND CUDA_NVCC_FLAGS --generate-code arch=compute_${compute_capability},code=sm_${compute_capability})
        endforeach ()

        # Enable fast math.
        set(CUDA_NVCC_FLAGS --use_fast_math ; ${CUDA_NVCC_FLAGS})

        # If on Windows, make it possible to enable CUDA debug information.
        if (MSVC_IDE)
            option(ENABLE_CUDA_DEBUGGING "Enable CUDA debugging?" OFF)
            if (ENABLE_CUDA_DEBUGGING)
                set(CUDA_NVCC_FLAGS -G; ${CUDA_NVCC_FLAGS})
            endif ()
        endif ()

        # If on Mac OS X 10.9 (Mavericks), make sure everything compiles and links using the correct C++ Standard Library.
        if (${CMAKE_SYSTEM} MATCHES "Darwin-13.")
            set(CUDA_HOST_COMPILER /usr/bin/clang)
            set(CUDA_NVCC_FLAGS -Xcompiler -stdlib=libstdc++; -Xlinker -stdlib=libstdc++; ${CUDA_NVCC_FLAGS})
        endif ()

        # If on Linux:
        if (${CMAKE_SYSTEM} MATCHES "Linux")
            # Make sure that C++11 support is enabled when compiling with nvcc. From CMake 3.5 onwards,
            # the host flag -std=c++11 is automatically propagated to nvcc. Manually setting it prevents
            # the project from building.
            if (${CMAKE_VERSION} VERSION_LESS 3.5)
                set(CUDA_NVCC_FLAGS -std=c++11; ${CUDA_NVCC_FLAGS})
            endif ()

            # Work around an Ubuntu 16.04 compilation error.
            if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 5.0)
                add_definitions(-D_FORCE_INLINES)
            endif ()
        endif ()

        # If not on Windows, disable some annoying nvcc warnings.
        if (NOT MSVC_IDE)
            set(CUDA_NVCC_FLAGS -Xcudafe "--diag_suppress=cc_clobber_ignored" ; -Xcudafe "--diag_suppress=set_but_not_used" ; ${CUDA_NVCC_FLAGS})
        endif ()
    endif ()
else ()
    add_definitions(-DCOMPILE_WITHOUT_CUDA)
endif ()
