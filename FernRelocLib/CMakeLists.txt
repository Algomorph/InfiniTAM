###################################
# CMakeLists.txt for FernRelocLib #
###################################

###########################
# Specify the target name #
###########################

set(targetname FernRelocLib)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

set(sources
FernConservatory.cpp
PoseDatabase.cpp
RelocDatabase.cpp
)

set(headers
FernConservatory.h
PixelUtils.h
PoseDatabase.h
RelocDatabase.h
Relocaliser.h
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

include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)