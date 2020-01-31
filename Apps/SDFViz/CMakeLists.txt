#####################################
# CMakeLists.txt for Apps/SDFViz    #
#####################################

###########################
# Specify the target name #
###########################

set(targetname SDFViz)


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

#############################
# Specify the project files #
#############################

set(sources
    #===core files===
    main.cpp
    SDFViz.cpp
    SDFVizInteractorStyle.cpp
    ../../ITMLib/Utils/Visualization/SceneSliceVisualizer3D.tpp
    ExplicitInstantiations.cpp
    HighlightVisualization.cpp
    #===helpers===
    )

set(headers
    #===core files===
    SDFViz.h
    SDFVizInteractorStyle.h
    ../../ITMLib/Utils/Visualization/SceneSliceVisualizer3D.h
    HighlightVisualization.h
    SDFVizGlobalDefines.h
    ../../ITMLib/Utils/Visualization/SceneSliceVisualizer3DCommon.h
    #===helpers===
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

target_link_libraries(${targetname} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib)
include(${PROJECT_SOURCE_DIR}/cmake/LinkFFmpeg.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkGLUT.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkLibRoyale.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenGL.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkOpenNI.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkRealSense.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkUVC.cmake)

#################################
# Extra libraries (this project only)
#################################
find_package(VTK 8.1 COMPONENTS
             vtkIOImage
             vtkChartsCore
             vtkCommonColor
             vtkCommonCore
             vtkCommonDataModel
             vtkFiltersSources
             vtkInteractionStyle
             vtkInteractionWidgets
             vtkRenderingContext2D
             vtkRenderingContextOpenGL2
             vtkRenderingCore
             vtkRenderingFreeType
             vtkRenderingOpenGL2
             vtkRenderingGL2PSOpenGL2
             vtkViewsContext2D
             REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include(${VTK_USE_FILE})
target_link_libraries(${targetname} ${VTK_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})