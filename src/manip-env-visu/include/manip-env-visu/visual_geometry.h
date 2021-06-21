#ifndef VISUAL_GEOMETRY_H
#define VISUAL_GEOMETRY_H

#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkPNGReader.h>
#include <vtkTexture.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf_model/link.h>
#include <manip-env-visu/utils.h>

#include <math.h>

#include <filesystem>

namespace mev
{
    class VisualGeometry
    {
        public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // mandatory when using eigen with fixed size matrices

        std::string mesh_path;
        std::string mesh_color;
        std::string texture_path;

        Eigen::Matrix4f geometry_world_pose;

        vtkSmartPointer<vtkPolyData> geometry_polydata;
        vtkNew<vtkPolyDataMapper> geometry_mapper;
        vtkSmartPointer<vtkTexture> geometry_texture;
        vtkSmartPointer<vtkPNGReader> geometry_texture_reader;
        vtkNew<vtkActor> geometry_actor;

        VisualGeometry(const std::string& model_path = "",
                       const std::string& color = "SlateGray",
                       const std::string& texture_path = "");
        bool setGeometryWorldPose(const Eigen::Matrix4f& geometry_world_pose);
        vtkSmartPointer<vtkActor> getGeometryActor();
        void addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer);
    };

    class URDFVisualGeometry : public VisualGeometry
    {
        public:

        urdf::MeshSharedPtr urdf_geometry;
        std::string urdf_path;

        URDFVisualGeometry(urdf::MeshSharedPtr urdf_geometry,
                           const std::string& urdf_path = "");
    };

}

















#endif