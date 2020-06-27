## https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake/56738858#56738858
## https://stackoverflow.com/a/56738858/3743145

## Get all properties that CMake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
## Convert command output into a CMake list
string(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
string(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")

list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

## Provided for potential debugging purposes on CMake targets
function(print_target_properties tgt)
    if (NOT TARGET ${tgt})
        message("There is no target named '${tgt}'")
        return()
    endif ()

    if (MSVC)
        foreach (prop ${CMAKE_PROPERTY_LIST})
            set(propval)
            if (prop MATCHES "^\\.*_<CONFIG>$")
                string(REPLACE "<CONFIG>" "Debug" prop_DEBUG ${prop})
                get_target_property(propval ${tgt} ${prop_DEBUG})
                if (propval)
                    message("${tgt} ${prop_DEBUG} = ${propval}")
                endif ()
                string(REPLACE "<RELEASE>" "Release" prop_RELEASE ${prop})
                get_target_property(propval ${tgt} ${prop_RELEASE})
                if (propval)
                    message("${tgt} ${prop_RELEASE} = ${propval}")
                endif ()
            else ()
                get_target_property(propval ${tgt} ${prop})
                if (propval)
                    message("${tgt} ${prop} = ${propval}")
                endif ()
            endif ()
        endforeach (prop)
    else ()
        foreach (prop ${CMAKE_PROPERTY_LIST})
            string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
            get_target_property(propval ${tgt} ${prop})
            if (propval)
                message("${tgt} ${prop} = ${propval}")
            endif ()
        endforeach (prop)
    endif ()
endfunction(print_target_properties)

function(handle_external_package)
    set(options PACKAGED)
    set(oneValueArgs NAME PREFERRED_SOURCE GIT_REPOSITORY GIT_TAG INCLUDE_TARGET LIBRARY_TARGET_POSIX LIBRARY_TARGET_MSVC_DEBUG LIBRARY_TARGET_MSVC_RELEASE ALTERNATIVE_LOCAL_NAME FORCE_PREFERRED_SOURCE)
    set(multiValueArgs CMAKE_ARGS LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES SOURCE_OPTIONS)
    cmake_parse_arguments(HEP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER ${HEP_NAME} HEP_UC_NAME)

    list(LENGTH HEP_SOURCE_OPTIONS SOURCE_OPTION_COUNT)
    if (SOURCE_OPTION_COUNT EQUAL 0)
        message(FATAL_ERROR "Source option count evaluates to zero, SOURCE_OPTIONS argument is ${HEP_SOURCE_OPTIONS}, please verify that it was set correctly.")
    endif ()
    list(FIND HEP_SOURCE_OPTIONS ${HEP_PREFERRED_SOURCE} PREFERRED_SOURCE_INDEX)
    if (PREFERRED_SOURCE_INDEX EQUAL -1)
        message(FATAL_ERROR "Unable to find the PREFERRED_SOURCE argument (${HEP_PREFERRED_SOURCE}) within the SOURCE_OPTIONS ${HEP_SOURCE_OPTIONS}, please verify that both were set correctly.")
    endif ()

    set(DEFAULT_SOURCE_OPTION_PRECEDENCE_ORDER FIND_LOCAL;BUILD_EXTERNAL;BUILD_PACKAGED)
    set(ORDERED_SOURCE_OPTIONS ${HEP_PREFERRED_SOURCE})
    if (NOT FORCE_PREFERRED_SOURCE)
        # determine order of source options
        foreach (DEFAULT_SOURCE_OPTION ${DEFAULT_SOURCE_OPTION_PRECEDENCE_ORDER})
            if (NOT DEFAULT_SOURCE_OPTION STREQUAL HEP_PREFERRED_SOURCE)
                list(FIND HEP_SOURCE_OPTIONS ${DEFAULT_SOURCE_OPTION} INDEX)
                if (NOT INDEX EQUAL -1)
                    list(APPEND ORDERED_SOURCE_OPTIONS ${DEFAULT_SOURCE_OPTION})
                endif ()
            endif ()
        endforeach (DEFAULT_SOURCE_OPTION)
    endif ()

    set(PACKAGE_READY FALSE)

    foreach (SOURCE_OPTION ${ORDERED_SOURCE_OPTIONS})
        if (NOT PACKAGE_READY)
            if (${SOURCE_OPTION} STREQUAL "FIND_LOCAL")
                if (HEP_ALTERNATIVE_LOCAL_NAME)
                    if (FORCE_PREFERRED_SOURCE)
                        find_package(${HEP_ALTERNATIVE_LOCAL_NAME} REQUIRED)
                    else ()
                        find_package(${HEP_ALTERNATIVE_LOCAL_NAME})
                    endif ()
                    if (${HEP_ALTERNATIVE_LOCAL_NAME}_FOUND)
                        add_library(${HEP_NAME} INTERFACE IMPORTED)
                        set(${HEP_NAME}_FOUND TRUE PARENT_SCOPE)
                        target_link_libraries(${HEP_NAME} INTERFACE ${HEP_ALTERNATIVE_LOCAL_NAME}::${HEP_ALTERNATIVE_LOCAL_NAME})
                        set(PACKAGE_READY TRUE)
                    else ()
                        message(WARNING "Unable to find the local package for external dependency ${HEP_NAME} trying alternative name \
'${HEP_ALTERNATIVE_LOCAL_NAME}'. Will try other sources if possible...")
                    endif ()
                else ()
                    if (FORCE_PREFERRED_SOURCE)
                        find_package(${HEP_NAME} REQUIRED)
                    else ()
                        find_package(${HEP_NAME})
                    endif ()
                    if (${HEP_ALTERNATIVE_LOCAL_NAME}_FOUND)
                        set(PACKAGE_READY TRUE)
                    else ()
                        message(WARNING "Unable to find the local package for external dependency ${HEP_NAME}. Will try other sources if possible...")
                    endif ()
                endif ()
            else ()
                set(DEPENDENCY_SOURCE_READY FALSE)
                set(${HEP_UC_NAME}_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/${HEP_NAME})
                if (${SOURCE_OPTION} STREQUAL "BUILD_EXTERNAL")
                    find_package(Git)
                    if (Git_FOUND)
                        externalproject_add(__${HEP_NAME}
                                            PREFIX ${${HEP_UC_NAME}_PREFIX}
                                            GIT_REPOSITORY ${HEP_GIT_REPOSITORY}
                                            GIT_TAG ${HEP_GIT_TAG}
                                            INSTALL_DIR ${${HEP_UC_NAME}_PREFIX}
                                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${HEP_UC_NAME}_PREFIX}
                                            ${HEP_CMAKE_ARGS}
                                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                                            )
                        set(DEPENDENCY_SOURCE_READY TRUE)
                    else()
                        message(WARNING "Unable to find Git executable to retrieve external dependency ${HEP_NAME} from a Git repository. Will try other sources if possible...")
                    endif ()
                elseif (${HEP_PACKAGED} AND ${SOURCE_OPTION} STREQUAL "BUILD_PACKAGED")
                    find_path(SOURCE_DIR CMakeLists.txt ${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${HEP_NAME})
                    if(SOURCE_DIR)
                        externalproject_add(__${HEP_NAME}
                                            PREFIX ${${HEP_UC_NAME}_PREFIX}
                                            PREFERRED_SOURCE_DIR ${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${HEP_NAME}
                                            INSTALL_DIR ${${HEP_UC_NAME}_PREFIX}
                                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${HEP_UC_NAME}_PREFIX}
                                            ${HEP_CMAKE_ARGS}
                                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                                            )
                        set(DEPENDENCY_SOURCE_READY TRUE)
                    else()
                        message(WARNING "Unable to find source folder (with CMakeLists.txt) for external dependency ${HEP_NAME} in \
${CMAKE_CURRENT_PREFERRED_SOURCE_DIR}/packaged/${HEP_NAME}. Will try other sources if possible...")
                    endif()
                endif ()
                if(DEPENDENCY_SOURCE_READY)
                    add_library(${HEP_NAME} STATIC IMPORTED GLOBAL)
                    set(${HEP_NAME}_FOUND TRUE PARENT_SCOPE)
                    add_dependencies(${HEP_NAME} __${HEP_NAME})
                    file(MAKE_DIRECTORY ${${HEP_UC_NAME}_PREFIX}/${HEP_INCLUDE_TARGET})
                    if (MSVC)
                        set_target_properties(${HEP_NAME} PROPERTIES
                                              IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX;RC"
                                              IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX;RC"
                                              INTERFACE_INCLUDE_DIRECTORIES ${${HEP_UC_NAME}_PREFIX}/${HEP_INCLUDE_TARGET}
                                              IMPORTED_LOCATION_RELWITHDEBINFO ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_MSVC_DEBUG}
                                              IMPORTED_LOCATION_RELEASE ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_MSVC_RELEASE}
                                              IMPORTED_LOCATION_DEBUG ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_MSVC_DEBUG}
                                              )
                        if (HEP_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES)
                            set_target_properties(${HEP_NAME} PROPERTIES
                                                  INTERFACE_LINK_LIBRARIES "${HEP_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES}")
                        endif ()
                    else ()
                        set_target_properties(${HEP_NAME} PROPERTIES
                                              INTERFACE_INCLUDE_DIRECTORIES ${${HEP_UC_NAME}_PREFIX}/${HEP_INCLUDE_TARGET}
                                              IMPORTED_LOCATION_RELWITHDEBINFO ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                              IMPORTED_LOCATION_RELEASE ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                              IMPORTED_LOCATION_DEBUG ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                              )
                    endif ()
                    set(PACKAGE_READY TRUE)
                endif()
            endif ()
        endif ()
    endforeach (SOURCE_OPTION)
    if(NOT PACKAGE_READY)
        message(FATAL_ERROR "Exhausted all available source options for external dependency ${HEP_NAME}.")
    endif()
endfunction()

function(provide_dependency_source_option DEPENDENCY_NAME)
    set(options FORCE_PREFERRED_BY_DEFAULT)
    set(oneValueArgs DEFAULT_INDEX)
    set(multiValueArgs OPTIONS)
    cmake_parse_arguments(PDSO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER ${DEPENDENCY_NAME} DEPENDENCY_NAME_UC)
    list(GET PDSO_OPTIONS ${PDSO_DEFAULT_INDEX} ${DEPENDENCY_NAME_UC}_SOURCE_DEFAULT_OPTION)
    set(${DEPENDENCY_NAME_UC}_PREFERRED_SOURCE "${${DEPENDENCY_NAME_UC}_SOURCE_DEFAULT_OPTION}" CACHE STRING
        "How to obtain the ${DEPENDENCY_NAME} dependency. Can be one of: [ ${DEPENDENCY_SOURCE_OPTIONS}]")
    set_property(CACHE ${DEPENDENCY_NAME_UC}_PREFERRED_SOURCE PROPERTY STRINGS ${PDSO_OPTIONS})
    set(${DEPENDENCY_NAME_UC}_SOURCE_OPTIONS ${PDSO_OPTIONS} PARENT_SCOPE)
    if (PDSO_FORCE_PREFERRED_BY_DEFAULT)
        set(FORCE_PREFERRED ON)
    else ()
        set(FORCE_PREFERRED OFF)
    endif ()
    option(${DEPENDENCY_NAME_UC}_FORCE_PREFERRED_SOURCE
           "Force the use of preferred source for dependency ${DEPENDENCY_NAME} (alternative options won't be tried if preferred doesn't succeed.)"
           ${FORCE_PREFERRED})
endfunction()