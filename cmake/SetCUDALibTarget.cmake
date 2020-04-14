##########################
# SetCUDALibTarget.cmake #
##########################

include(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

# TODO: test that with MSVC, the desired effect is achieved with CMake v. <3.9 (CMake boolean/if syntax is kind-of obscure)
if (${CMAKE_VERSION} VERSION_LESS 3.8 OR (MSVC_IDE AND ${CMAKE_VERSION} VERSION_LESS 3.9))
    if (WITH_CUDA)
        cuda_add_library(${targetname} STATIC ${sources} ${headers} ${templates})
    else ()
        add_library(${targetname} STATIC ${sources} ${headers} ${templates})
    endif ()
else ()
    if (WITH_CUDA)
        enable_language(CUDA)
    endif ()
    add_library(${targetname} STATIC ${sources} ${headers} ${templates})
    target_include_directories(${targetname} PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif ()
