include(SourceOptions)

function(provide_dependency_source_option DEPENDENCY_NAME)
    set(options FORCE_PREFERRED_BY_DEFAULT)
    set(oneValueArgs DEFAULT_INDEX)
    set(multiValueArgs OPTIONS)
    cmake_parse_arguments(PDSO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER ${DEPENDENCY_NAME} DEPENDENCY_NAME_UC)
    list(GET PDSO_OPTIONS ${PDSO_DEFAULT_INDEX} ${DEPENDENCY_NAME_UC}_SOURCE_DEFAULT_OPTION)
    set(${DEPENDENCY_NAME_UC}_PREFERRED_SOURCE "${${DEPENDENCY_NAME_UC}_SOURCE_DEFAULT_OPTION}" CACHE STRING
        "How to obtain the ${DEPENDENCY_NAME} dependency. Can be one of: [ ${PDSO_OPTIONS} ]")
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