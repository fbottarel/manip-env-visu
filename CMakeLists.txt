cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

project(manip-env-visu VERSION 0.1.0 DESCRIPTION "Simple lib for manipulation environment visualization")
set(LIB_TARGET_NAME manip-env-visu)
# Find the urdfdom dependency

find_package(urdfdom REQUIRED)
if (${urdfdom_FOUND})
  message(STATUS "Found urdfdom package")
else (${urdfdom_FOUND})
  message(FATAL_ERROR "Could not locate urdfdom package")
endif()

# Find the eigen dependency

find_package(Eigen3 REQUIRED)
if (${Eigen3_FOUND})
  message(STATUS "Found eigen3 package")
else (${Eigen3_FOUND})
  message(FATAL_ERROR "Could not locate eigen3 package")
endif()

# Find VTK modules

find_package(VTK COMPONENTS
  vtkCommonColor
  vtkCommonCore
  vtkCommonDataModel
  vtkFiltersGeneral
  vtkFiltersSources
  vtkIOGeometry
  vtkIOLegacy
  vtkIOPLY
  vtkIOXML
  vtkInteractionStyle
  vtkRenderingAnnotation
  vtkRenderingContextOpenGL2
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingGL2PSOpenGL2
  vtkRenderingOpenGL2
  QUIET
)
if (${VTK_FOUND})
  message(STATUS "Found VTK package")
  message(STATUS "VTK_VERSION: ${VTK_VERSION}")
else (${VTK_FOUND})
  message(FATAL_ERROR "Could not locate VTK package")
endif()
include(${VTK_USE_FILE})

# Define lists for header and source files

set(${LIB_TARGET_NAME}_HDR
    include/gripper.h
    include/joint.h
    include/link.h
    include/manipulation_env.h
    include/utils.h
    include/visual_geometry.h
    )

set(${LIB_TARGET_NAME}_SRC
    src/gripper.cpp
    src/joint.cpp
    src/link.cpp
    src/manipulation_env.cpp
    src/utils.cpp
    src/visual_geometry.cpp
    )

# Declare shared library

add_library(${LIB_TARGET_NAME} SHARED
            ${${LIB_TARGET_NAME}_SRC}
            ${${LIB_TARGET_NAME}_HDR}
            )

# Set headers dir as include

target_include_directories(${LIB_TARGET_NAME}
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )

# Link dependencies for the shared lib

message("Processing library")
target_link_libraries(${LIB_TARGET_NAME}
                      PUBLIC
                      Eigen3::Eigen
                      ${urdfdom_LIBRARIES}
                      ${VTK_LIBRARIES}
                      )

# Compile examples

add_subdirectory(examples)

