##########################
# SetCUDAAppTarget.cmake #
##########################


# TODO: test that with MSVC, the desired effect is achieved with CMake v. <3.9 (CMake boolean/if syntax is kind-of obscure)
if (${CMAKE_VERSION} VERSION_LESS 3.8 OR (MSVC_IDE AND ${CMAKE_VERSION} VERSION_LESS 3.9))
    if (WITH_CUDA)
        cuda_add_executable(${targetname} ${sources} ${headers} ${templates})
    else ()
        add_executable(${targetname} ${sources} ${headers} ${templates})
    endif ()
else ()
    if (WITH_CUDA)
        enable_language(CUDA)
    endif ()
    add_executable(${targetname} ${sources} ${headers} ${templates})
endif ()

# TODO: not sure if this is still needed, it looks like a work-around for a CMake/MSVC bug that may have been resolved
if (MSVC_IDE)
    set_target_properties(${targetname} PROPERTIES LINK_FLAGS_DEBUG "/DEBUG")
endif ()