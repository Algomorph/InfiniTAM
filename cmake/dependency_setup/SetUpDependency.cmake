include(DownloadPrebuiltPackage)
include(SourceOptions)
include(HandleLocalPackage)


function(set_up_dependency)
    set(options PACKAGED)
    set(oneValueArgs NAME PREFERRED_SOURCE GIT_REPOSITORY GIT_TAG SOURCE_SUBDIR INCLUDE_TARGET LIBRARY_TARGET_POSIX LIBRARY_TARGET_MSVC_DEBUG LIBRARY_TARGET_MSVC_RELEASE ALTERNATIVE_LOCAL_NAME FORCE_PREFERRED_SOURCE)
    set(multiValueArgs CMAKE_ARGS LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES SOURCE_OPTIONS COMPONENTS OPTIONAL_COMPONENTS)
    cmake_parse_arguments(SUD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER ${SUD_NAME} SUD_UC_NAME)

    list(LENGTH SUD_SOURCE_OPTIONS SOURCE_OPTION_COUNT)
    if (SOURCE_OPTION_COUNT EQUAL 0)
        message(FATAL_ERROR "Source option count evaluates to zero, SOURCE_OPTIONS argument is ${SUD_SOURCE_OPTIONS}, please verify that it was set correctly.")
    endif ()

    list(FIND SUD_SOURCE_OPTIONS ${SUD_PREFERRED_SOURCE} PREFERRED_SOURCE_INDEX)
    if (PREFERRED_SOURCE_INDEX EQUAL -1)
        message(FATAL_ERROR "Unable to find the PREFERRED_SOURCE argument (${SUD_PREFERRED_SOURCE}) within the SOURCE_OPTIONS ${SUD_SOURCE_OPTIONS}, please verify that both were set correctly.")
    endif ()

    set(DEFAULT_SOURCE_OPTION_PRECEDENCE_ORDER ${FIND_LOCAL_SOURCE};${BUILD_EXTERNAL_SOURCE};${BUILD_PACKAGED_SOURCE};${DOWNLOAD_PREBUILT_SOURCE})
    set(ORDERED_SOURCE_OPTIONS ${SUD_PREFERRED_SOURCE})
    if (NOT SUD_FORCE_PREFERRED_SOURCE)
        # determine order of source options
        foreach (DEFAULT_SOURCE_OPTION ${DEFAULT_SOURCE_OPTION_PRECEDENCE_ORDER})
            if (NOT DEFAULT_SOURCE_OPTION STREQUAL SUD_PREFERRED_SOURCE)
                list(FIND SUD_SOURCE_OPTIONS ${DEFAULT_SOURCE_OPTION} INDEX)
                if (NOT INDEX EQUAL -1)
                    list(APPEND ORDERED_SOURCE_OPTIONS ${DEFAULT_SOURCE_OPTION})
                endif ()
            endif ()
        endforeach (DEFAULT_SOURCE_OPTION)
    endif ()

    set(PACKAGE_READY FALSE)
    
    foreach (SOURCE_OPTION ${ORDERED_SOURCE_OPTIONS})
        if (NOT PACKAGE_READY)
            if (SOURCE_OPTION STREQUAL FIND_LOCAL_SOURCE)
                handle_local_package(
                    NAME ${SUD_NAME}
                    ALTERNATIVE_LOCAL_NAME ${SUD_ALTERNATIVE_LOCAL_NAME}
                    REQUIRE_LOCAL_FOUND ${SUD_FORCE_PREFERRED_SOURCE}
                    COMPONENTS ${SUD_COMPONENTS}
                    OPTIONAL_COMPONENTS ${SUD_OPTIONAL_COMPONENTS}
                )
                if(PACKAGE_READY)
                    set(${SUD_UC_NAME}_SOURCE ${SOURCE_OPTION} PARENT_SCOPE)
                else ()
                    message(WARNING "Unable to find the local package for dependency '${SUD_NAME}' (trying name \
'${SUD_ALTERNATIVE_LOCAL_NAME}'). Will try other sources if possible...")
                endif ()
            elseif (SOURCE_OPTION STREQUAL BUILD_PACKAGED_SOURCE OR SOURCE_OPTION STREQUAL BUILD_EXTERNAL_SOURCE)
                set(DEPENDENCY_SOURCE_READY FALSE)
                set(${SUD_UC_NAME}_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/${SUD_NAME})
                if (${SOURCE_OPTION} STREQUAL "BUILD_EXTERNAL")
                    find_package(Git)
                    if (Git_FOUND)
                        externalproject_add(__${SUD_NAME}
                                            PREFIX ${${SUD_UC_NAME}_PREFIX}
                                            GIT_REPOSITORY ${SUD_GIT_REPOSITORY}
                                            GIT_TAG ${SUD_GIT_TAG}
                                            INSTALL_DIR ${${SUD_UC_NAME}_PREFIX}
                                            SOURCE_SUBDIR ${SUD_SOURCE_SUBDIR}
                                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${SUD_UC_NAME}_PREFIX}
                                            ${SUD_CMAKE_ARGS}
                                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                                            )
                        set(DEPENDENCY_SOURCE_READY TRUE)
                    else()
                        message(WARNING "Unable to find Git executable to retrieve dependency ${SUD_NAME} from a Git repository. Will try other sources if possible...")
                    endif ()
                elseif (${SUD_PACKAGED} AND ${SOURCE_OPTION} STREQUAL "BUILD_PACKAGED")
                    find_path(SOURCE_DIR CMakeLists.txt ${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${SUD_NAME})
                    if(SOURCE_DIR)
                        externalproject_add(__${SUD_NAME}
                                            PREFIX ${${SUD_UC_NAME}_PREFIX}
                                            PREFERRED_SOURCE_DIR ${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${SUD_NAME}
                                            INSTALL_DIR ${${SUD_UC_NAME}_PREFIX}
                                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${SUD_UC_NAME}_PREFIX}
                                            ${SUD_CMAKE_ARGS}
                                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                                            )
                        set(DEPENDENCY_SOURCE_READY TRUE)
                    else()
                        message(WARNING "Unable to find source folder (with CMakeLists.txt) for dependency '${SUD_NAME}' in \
${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${SUD_NAME}. Will try other sources if possible...")
                    endif()
                endif ()
                if(DEPENDENCY_SOURCE_READY)
                    add_library(${SUD_NAME} STATIC IMPORTED GLOBAL)
                    set(${SUD_NAME}_FOUND TRUE PARENT_SCOPE)
                    add_dependencies(${SUD_NAME} __${SUD_NAME})
                    file(MAKE_DIRECTORY ${${SUD_UC_NAME}_PREFIX}/${SUD_INCLUDE_TARGET})
                    if (MSVC)
                        set_target_properties(${SUD_NAME} PROPERTIES
                                              IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX;RC"
                                              IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX;RC"
                                              INTERFACE_INCLUDE_DIRECTORIES ${${SUD_UC_NAME}_PREFIX}/${SUD_INCLUDE_TARGET}
                                              IMPORTED_LOCATION_RELWITHDEBINFO ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_MSVC_DEBUG}
                                              IMPORTED_LOCATION_RELEASE ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_MSVC_RELEASE}
                                              IMPORTED_LOCATION_DEBUG ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_MSVC_DEBUG}
                                              )
                        if (SUD_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES)
                            set_target_properties(${SUD_NAME} PROPERTIES
                                                  INTERFACE_LINK_LIBRARIES "${SUD_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES}")
                        endif ()
                    else ()
                        set_target_properties(${SUD_NAME} PROPERTIES
                                              INTERFACE_INCLUDE_DIRECTORIES ${${SUD_UC_NAME}_PREFIX}/${SUD_INCLUDE_TARGET}
                                              IMPORTED_LOCATION_RELWITHDEBINFO ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_POSIX}
                                              IMPORTED_LOCATION_RELEASE ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_POSIX}
                                              IMPORTED_LOCATION_DEBUG ${${SUD_UC_NAME}_PREFIX}/${SUD_LIBRARY_TARGET_POSIX}
                                              )
                    endif ()
                    set(PACKAGE_READY TRUE)
                    set(${SUD_UC_NAME}_SOURCE ${SOURCE_OPTION} PARENT_SCOPE)
                endif()
            elseif(SOURCE_OPTION STREQUAL DOWNLOAD_PREBUILT_SOURCE)
                if(MSVC)
                    download_prebuilt_package_msvc(
                        NAME ${SUD_NAME}
                        ALTERNATIVE_LOCAL_NAME ${SUD_ALTERNATIVE_LOCAL_NAME}
                        REQUIRE_FOUND ${SUD_FORCE_PREFERRED_SOURCE}
                        COMPONENTS ${SUD_COMPONENTS}
                        OPTIONAL_COMPONENTS ${SUD_OPTIONAL_COMPONENTS}
                    )
                    if(PACKAGE_READY)
                        set(${SUD_UC_NAME}_SOURCE ${SOURCE_OPTION} PARENT_SCOPE)
                    else ()
                        message(WARNING "Unable to find the downloaded prebuilt package for dependency '${SUD_NAME}' (trying name \
'${SUD_ALTERNATIVE_LOCAL_NAME}'). Possibly, either download is not set up correctly or links / content are broken/altered. \
Will try other sources if possible...")
                    endif ()
                else()
                    message(FATAL_ERROR "DOWNLOAD_PREBUILT_SOURCE option chosen for dependency '${SUD_NAME}'. This option is not currently supported for CMake generators other than Michrosoft Visual Studio.")
                endif()
            endif ()
        endif (NOT PACKAGE_READY)
    endforeach (SOURCE_OPTION)
    if(NOT PACKAGE_READY)
        message(FATAL_ERROR "Exhausted all available source options for dependency '${SUD_NAME}'.")
    endif()
endfunction()