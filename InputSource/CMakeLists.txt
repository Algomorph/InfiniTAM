##################################
# CMakeLists.txt for InputSource #
##################################

###########################
# Specify the target name #
###########################

SET(targetname InputSource)

#####################################
# Offer support for input mirroring #
#####################################

include(${PROJECT_SOURCE_DIR}/cmake/OfferInputMirroring.cmake)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseKinect2.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenGL.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

set(sources
CompositeImageSourceEngine.cpp
FFMPEGReader.cpp
FFMPEGWriter.cpp
ImageSourceEngine.cpp
IMUSourceEngine.cpp
Kinect2Engine.cpp
LibUVCEngine.cpp
OpenNIEngine.cpp
PicoFlexxEngine.cpp
RealSenseEngine.cpp
)

SET(headers
CompositeImageSourceEngine.h
FFMPEGReader.h
FFMPEGWriter.h
ImageSourceEngine.h
IMUSourceEngine.h
Kinect2Engine.h
LibUVCEngine.h
OpenNIEngine.h
PicoFlexxEngine.h
RealSenseEngine.h
)

#############################
# Specify the source groups #
#############################

source_group("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

include(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)


##########################################
# Link external dependency libraries     #
##########################################
target_link_libraries(InputSource PUBLIC ffmpeg)

include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
