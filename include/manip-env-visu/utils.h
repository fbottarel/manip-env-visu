#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <filesystem>

#include <urdf_model/pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

/// Check for file existence
bool fileExists(const std::string filename);

/// Pose to Eigen matrix
Eigen::Matrix4f getHomogeneousTransform(const urdf::Pose &pose);

/// Read mesh from file
vtkSmartPointer<vtkPolyData> readPolyDataFromFile(std::string const& mesh_path);

/// Get directory from path to file
std::string getDirPathFromFilePath(const std::string& dir_path);

#endif