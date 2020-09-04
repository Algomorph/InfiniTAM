#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################

###########################
# Specify the target name #
###########################

set(targetname InfiniTAM)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)


#############################
# Specify the project files #
#############################

set(
        sources

        InfiniTAM.cpp
        UIEngine.cpp
        UIEngine_GlutFunctions.cpp
        CreateDefaultImageSource.cpp
)

set(
        headers

        UIEngine.h
        prettyprint.hpp
        CreateDefaultImageSource.h
)

#############################
# Specify the source groups #
#############################

source_group("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

add_executable(${targetname} ${sources} ${headers})

#################################
# Specify the libraries to link #
#################################

target_link_libraries(${targetname} PUBLIC glut InputSource ITMLib OpenGL::GL Boost::program_options)

include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
