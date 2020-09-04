#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################

###########################
# Specify the target name #
###########################

SET(targetname InfiniTAM_cli)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

set(sources
    InfiniTAM_cli.cpp
    CLIEngine.cpp
    CreateDefaultImageSource.cpp
)

set(headers
    CLIEngine.h
    CreateDefaultImageSource.h
)

#############################
# Specify the source groups #
#############################

source_group("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################
add_library(${targetname} ${sources} ${headers})
#################################
# Specify the libraries to link #
#################################

target_link_libraries(${targetname} PRIVATE MiniSlamGraphLib ORUtils FernRelocLib ITMLib InputSource OpenGL::GL)

include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)