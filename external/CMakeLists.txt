include(ExternalProject)
#########################################################
# LOG4CPLUS
#########################################################
#TODO (make into function for an arbitrary <depenency_name> (currently, log4cplus is the dependency_name)

function(handle_external_package)
    set(options PACKAGED)
    set(oneValueArgs NAME SOURCE GIT_REPOSITORY GIT_TAG INCLUDE_TARGET LIBRARY_TARGET_POSIX LIBRARY_TARGET_MSVC_DEBUG LIBRARY_TARGET_MSVC_RELEASE ALTERNATIVE_LOCAL_NAME)
    set(multiValueArgs CMAKE_ARGS LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES)
    cmake_parse_arguments(HEP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER ${HEP_NAME} HEP_UC_NAME)

    if (${HEP_SOURCE} STREQUAL "FIND_LOCAL")
        if(HEP_ALTERNATIVE_LOCAL_NAME)
            find_package(${HEP_ALTERNATIVE_LOCAL_NAME} REQUIRED)
            add_library(${HEP_NAME} INTERFACE IMPORTED)
            set(${HEP_NAME}_FOUND TRUE PARENT_SCOPE)
            target_link_libraries(${HEP_NAME} INTERFACE ${HEP_ALTERNATIVE_LOCAL_NAME}::${HEP_ALTERNATIVE_LOCAL_NAME})
        else()
            find_package(${HEP_NAME} REQUIRED)
        endif()
    else ()
        set(${HEP_UC_NAME}_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/${HEP_NAME})
        if (${HEP_SOURCE} STREQUAL "BUILD_EXTERNAL")
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

        elseif (${HEP_PACKAGED} AND ${HEP_SOURCE} STREQUAL "BUILD_PACKAGED")
            externalproject_add(__${HEP_NAME}
                                PREFIX ${${HEP_UC_NAME}_PREFIX}
                                SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/packaged/${HEP_NAME}
                                INSTALL_DIR ${${HEP_UC_NAME}_PREFIX}
                                CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${HEP_UC_NAME}_PREFIX}
                                ${HEP_CMAKE_ARGS}
                                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                                -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                                -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                                -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                                -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                                )
        endif ()
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
            if(HEP_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES)
                set_target_properties(${HEP_NAME} PROPERTIES
                                      INTERFACE_LINK_LIBRARIES "${HEP_LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES}")
            endif()
        else ()
            set_target_properties(${HEP_NAME} PROPERTIES
                                  INTERFACE_INCLUDE_DIRECTORIES ${${HEP_UC_NAME}_PREFIX}/${HEP_INCLUDE_TARGET}
                                  IMPORTED_LOCATION_RELWITHDEBINFO ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                  IMPORTED_LOCATION_RELEASE ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                  IMPORTED_LOCATION_DEBUG ${${HEP_UC_NAME}_PREFIX}/${HEP_LIBRARY_TARGET_POSIX}
                                  )
        endif ()
    endif ()
endfunction()


handle_external_package(NAME log4cplus
                        SOURCE ${LOG4CPLUS_SOURCE}
                        GIT_REPOSITORY https://github.com/log4cplus/log4cplus.git
                        GIT_TAG 3b9d98fb5fd72298745cf21620ace8f2b6d770ad
                        CMAKE_ARGS -DWITH_UNIT_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -DUNICODE=OFF -DLOG4CPLUS_BUILD_TESTING=OFF -DLOG4CPLUS_BUILD_LOGGINGSERVER=OFF
                        PACKAGED
                        INCLUDE_TARGET include
                        LIBRARY_TARGET_MSVC_DEBUG lib/log4cplusSD.lib
                        LIBRARY_TARGET_MSVC_RELEASE lib/log4cplusS.lib
                        LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES "ws2_32;advapi32" 
                        LIBRARY_TARGET_POSIX lib/liblog4cplusS.a
                        )

handle_external_package(NAME zlib
                        SOURCE ${ZLIB_SOURCE}
                        GIT_REPOSITORY https://github.com/madler/zlib.git
                        GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f
                        PACKAGED
                        INCLUDE_TARGET include
                        LIBRARY_TARGET_MSVC_DEBUG lib/zlibd.lib
                        LIBRARY_TARGET_MSVC_RELEASE lib/zlib.lib
                        LIBRARY_TARGET_POSIX lib/libz.a
                        ALTERNATIVE_LOCAL_NAME ZLIB
                        )
#TODO
#handle_external_package(NAME png
#                        SOURCE ${PNG_SOURCE}
#                        GIT_REPOSITORY https://github.com/madler/zlib.git
#                        GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f
#                        PACKAGED
#                        INCLUDE_TARGET include
#                        LIBRARY_TARGET_MSVC_DEBUG lib/zlibd.lib
#                        LIBRARY_TARGET_MSVC_RELEASE lib/zlib.lib
#                        LIBRARY_TARGET_POSIX lib/libz.a
#                        ALTERNATIVE_LOCAL_NAME ZLIB
#                        )

add_library(tinyply ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.h ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.cpp)
target_include_directories(tinyply INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)

add_library(zstr ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.hpp ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/strict_fstream.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.cpp)
target_link_libraries(zstr zlib)
target_include_directories(zstr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)