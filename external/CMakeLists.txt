include(ExternalProject)
#########################################################
# LOG4CPLUS
#########################################################
#TODO (make into function for an arbitrary <depenency_name> (currently, log4cplus is the dependency_name)

if (LOG4CPLUS_SOURCE STREQUAL "FIND_LOCAL")
    find_package(log4cplus REQUIRED)
else ()
    set(LOG4CPLUS_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/log4cplus)
    if (LOG4CPLUS_SOURCE STREQUAL "BUILD_EXTERNAL")
        externalproject_add(__log4cplus
                            PREFIX ${LOG4CPLUS_PREFIX}
                            GIT_REPOSITORY https://github.com/log4cplus/log4cplus.git
                            GIT_TAG 3b9d98fb5fd72298745cf21620ace8f2b6d770ad
                            INSTALL_DIR ${LOG4CPLUS_PREFIX}
                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${LOG4CPLUS_PREFIX} -DWITH_UNIT_TESTS=OFF
                            -DBUILD_SHARED_LIBS=OFF
                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                            )

    elseif (LOG4CPLUS_SOURCE STREQUAL "BUILD_PACKAGED")
        externalproject_add(__log4cplus
                            PREFIX ${LOG4CPLUS_PREFIX}
                            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/packaged/log4cplus
                            INSTALL_DIR ${LOG4CPLUS_PREFIX}
                            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${LOG4CPLUS_PREFIX} -DWITH_UNIT_TESTS=OFF
                            -DBUILD_SHARED_LIBS=OFF
                            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                            -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE}
                            -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
                            -DCMAKE_C_FLAGS_MINSIZEREL=${CMAKE_C_FLAGS_MINSIZEREL}
                            -DCMAKE_C_FLAGS_RELWITHDEBINFO=${CMAKE_C_FLAGS_RELWITHDEBINFO}
                            )
    endif ()
    add_library(log4cplus STATIC IMPORTED GLOBAL)
    set(log4cplus_FOUND TRUE PARENT_SCOPE)
    add_dependencies(log4cplus __log4cplus)
    file(MAKE_DIRECTORY ${LOG4CPLUS_PREFIX}/include)
    if (MSVC)
        set_target_properties(log4cplus PROPERTIES
                              INTERFACE_INCLUDE_DIRECTORIES ${LOG4CPLUS_PREFIX}/include
                              IMPORTED_LOCATION_RELWITHDEBINFO ${LOG4CPLUS_PREFIX}/lib/log4cplusS.lib
                              IMPORTED_LOCATION_RELEASE ${LOG4CPLUS_PREFIX}/lib/log4cplusS.lib
                              IMPORTED_LOCATION_DEBUG ${LOG4CPLUS_PREFIX}/lib/log4cplusS.lib
                              )
    else ()
        set_target_properties(log4cplus PROPERTIES
                              INTERFACE_INCLUDE_DIRECTORIES ${LOG4CPLUS_PREFIX}/include
                              IMPORTED_LOCATION_RELWITHDEBINFO ${LOG4CPLUS_PREFIX}/lib/liblog4cplusS.a
                              IMPORTED_LOCATION_RELEASE ${LOG4CPLUS_PREFIX}/lib/liblog4cplusS.a
                              IMPORTED_LOCATION_DEBUG ${LOG4CPLUS_PREFIX}/lib/liblog4cplusS.a
                              )
    endif ()
endif ()

add_library(tinyply ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.h ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.cpp)
target_include_directories(tinyply INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)