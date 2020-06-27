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

include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseFFmpeg.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseGLUT.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseVTK.cmake)


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

include(${PROJECT_SOURCE_DIR}/cmake/SetCUDAAppTarget.cmake)

#################################
# Specify the libraries to link #
#################################

target_link_libraries(${targetname} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib glut)

include(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)

target_link_libraries(${targetname} ${VTK_LIBRARIES})