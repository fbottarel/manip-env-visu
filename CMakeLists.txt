cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(manip-env-visu VERSION 0.1.0 DESCRIPTION "Simple lib for manipulation environment visualization" LANGUAGES CXX)

set(LIB_TARGET_NAME manip-env-visu)
set(CMAKE_CXX_STANDARD 17)
set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Use GNU installation dirs as standard

include(GNUInstallDirs)

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

find_package(VTK 8.2 REQUIRED COMPONENTS
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
    include/manip-env-visu/gripper.h
    include/manip-env-visu/joint.h
    include/manip-env-visu/link.h
    include/manip-env-visu/manipulation_env.h
    include/manip-env-visu/utils.h
    include/manip-env-visu/visual_geometry.h
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
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
                           )

# Set target properties

set_target_properties(${LIB_TARGET_NAME} PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      SOVERSION 1
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

# Declare install targets and paths

install(TARGETS ${PROJECT_NAME} EXPORT ${LIB_TARGET_NAME}-config
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
install(DIRECTORY
        include/
        DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}
        )
install(EXPORT
        ${LIB_TARGET_NAME}-config
        DESTINATION
        share/${LIB_TARGET_NAME}/cmake
        )
export(TARGETS
       ${LIB_TARGET_NAME}
       FILE
       ${LIB_TARGET_NAME}-config.cmake
       )

