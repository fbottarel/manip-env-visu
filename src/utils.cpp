#include <manip-env-visu/utils.h>

bool fileExists(const std::string filename)
{
    std::ifstream filestream(filename);
    return filestream.good();
}

Eigen::Matrix4f getHomogeneousTransform(const urdf::Pose &pose)
{
    Eigen::Quaternion<float> rotation_quat(pose.rotation.w,
                                           pose.rotation.x,
                                           pose.rotation.y,
                                           pose.rotation.z);

    Eigen::Matrix3f rotation = rotation_quat.toRotationMatrix();
    Eigen::Vector3f translation(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Matrix4f transformation;
    transformation.setIdentity();
    transformation.block<3,3>(0,0) = rotation;
    transformation.block<3,1>(0,3) = translation;

    return transformation;
}

vtkSmartPointer<vtkPolyData> readPolyDataFromFile(std::string const& fileName)
{
  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension = "";
  if (fileName.find_last_of(".") != std::string::npos)
  {
    extension = fileName.substr(fileName.find_last_of("."));
  }
  // Make the extension lowercase
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  if (extension == ".ply")
  {
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".vtp")
  {
    vtkNew<vtkXMLPolyDataReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".obj")
  {
    vtkNew<vtkOBJReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".stl")
  {
    vtkNew<vtkSTLReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".vtk")
  {
    vtkNew<vtkPolyDataReader> reader;
    reader->SetFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".g")
  {
    vtkNew<vtkBYUReader> reader;
    reader->SetGeometryFileName(fileName.c_str());
    reader->Update();
    polyData = reader->GetOutput();
  }
  else
  {
    // Return a polydata sphere if the extension is unknown.
    vtkNew<vtkSphereSource> source;
    source->SetThetaResolution(20);
    source->SetPhiResolution(11);
    source->Update();
    polyData = source->GetOutput();
  }
  return polyData;
}

std::string getDirPathFromFilePath(const std::string& dir_path)
{
  std::filesystem::path input_path(dir_path);
  if (std::filesystem::is_directory(input_path))
  {
    return dir_path;
  }
  else
  {
    return input_path.parent_path().string();
  }
}