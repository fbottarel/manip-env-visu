#include <manip-env-visu/visual_geometry.h>

namespace mev
{
    VisualGeometry::VisualGeometry(const std::string& model_path,
                                   const std::string& color,
                                   const std::string& texture_path)
    : mesh_path {model_path},
      mesh_color {color},
      texture_path {texture_path}
    {
        // Standard VTK procedure to produce a polydata actor
        geometry_polydata = readPolyDataFromFile(mesh_path);
        geometry_mapper->SetInputData(geometry_polydata);
        geometry_actor->SetMapper(geometry_mapper);
        geometry_actor->SetOrigin(0,0,0);
        if (std::filesystem::exists(texture_path))
        {
            geometry_texture_reader = vtkSmartPointer<vtkPNGReader>::New();
            geometry_texture = vtkSmartPointer<vtkTexture>::New();
            geometry_texture_reader->SetFileName(texture_path.c_str());
            geometry_texture_reader->Update();
            geometry_texture->SetInputConnection(geometry_texture_reader->GetOutputPort());
            geometry_actor->SetTexture(geometry_texture);
        }
        else
        {
            std::cout << "[WARNING] texture file " << texture_path << " does not exist" << std::endl;
            vtkNew<vtkNamedColors> colors;
            geometry_actor->GetProperty()->SetColor(colors->GetColor3d(mesh_color).GetData());
        }
        // Geometry lays in the world origin ref frame by default
        geometry_world_pose = Eigen::Matrix4f::Identity();
    }

    bool VisualGeometry::setGeometryWorldPose(const Eigen::Matrix4f& geometry_world_pose)
    {
        this->geometry_world_pose = geometry_world_pose;

        Eigen::Matrix3f rotation = geometry_world_pose.block<3,3>(0,0);
        Eigen::Vector4f position = geometry_world_pose.col(3);

        if (rotation.determinant() != 1.0f)
        {
            std::cout << "Rotation matrix has determinant != 1" << std::endl;
            return false;
        }
        if (geometry_world_pose(3,3) != 1.0f)
        {
            std::cout << "4x4 matrix is not homogeneous" << std::endl;
            return false;
        }

        geometry_actor->SetPosition(position[0],
                                    position[1],
                                    position[2]);
        // Express rotation as zxy
        Eigen::Vector3f rotation_euler_deg = rotation.eulerAngles(2,0,1) * 180/M_PI;
        // VTK requires rotations input as x y and z, in degrees
        geometry_actor->SetOrientation(rotation_euler_deg[1],
                                       rotation_euler_deg[2],
                                       rotation_euler_deg[0]);
        return true;
    }

    vtkSmartPointer<vtkActor> VisualGeometry::getGeometryActor()
    {
        return geometry_actor;
    }

    void VisualGeometry::addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer)
    {
        // std::cout << "geometry is at position " << geometry_actor->GetPosition()[0] << " " << geometry_actor->GetPosition()[1] << " " << geometry_actor->GetPosition()[2] << std::endl;
        renderer->AddActor(geometry_actor);
    }

    /// the forward slash here is dirty, it should be fixed
    URDFVisualGeometry::URDFVisualGeometry(urdf::MeshSharedPtr urdf_geometry, const std::string& urdf_path)
    : VisualGeometry(getDirPathFromFilePath(urdf_path) + "/" + urdf_geometry->filename),
      urdf_geometry {urdf_geometry},
      urdf_path {urdf_path}
    {
    }

}