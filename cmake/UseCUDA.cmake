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
if (NOT WITH_CUDA)
    add_definitions(-DCOMPILE_WITHOUT_CUDA)
endif ()
