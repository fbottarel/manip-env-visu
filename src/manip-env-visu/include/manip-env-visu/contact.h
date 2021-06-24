#ifndef CONTACT_H
#define CONTACT_H

#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkConeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <manip-env-visu/utils.h>
#include <manip-env-visu/visual_geometry.h>

namespace mev
{
    class Contact
    {
        private:

        Eigen::Matrix4f contact_normal_pose;

        bool display_cone;
        bool display_force;
        bool display_contact_point;

        vtkSmartPointer<vtkConeSource> friction_cone_source;
        vtkSmartPointer<vtkArrowSource> force_vector_source;
        vtkSmartPointer<vtkSphereSource> point_source;

        vtkSmartPointer<vtkTransformPolyDataFilter> force_transform_filter;

        vtkSmartPointer<vtkPolyDataMapper> cone_mapper;
        vtkSmartPointer<vtkPolyDataMapper> force_mapper;
        vtkSmartPointer<vtkPolyDataMapper> point_mapper;

        vtkSmartPointer<vtkActor> cone_actor;
        vtkSmartPointer<vtkActor> force_actor;
        vtkSmartPointer<vtkActor> point_actor;

        float force_norm{0.05};
        float friction_coeff;
        float point_size{0.002};

        vtkSmartPointer<vtkNamedColors> colors;

        std::array<unsigned char, 4> cone_color{{0, 255, 0, 100}};
        std::array<unsigned char, 4> force_color{{255, 255, 0, 100}};
        std::array<unsigned char, 4> point_color{{0, 0, 255, 100}};

        void setActorColor(const vtkSmartPointer<vtkActor> actor,
                           const std::array<unsigned char, 4>& color);

        public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // mandatory when using eigen with fixed size matrices

        Contact(const Eigen::Matrix4f& contact_normal,
                float friction_coeff = 0.0,
                bool display_cone = false,
                bool display_force = false,
                bool display_contact_point = true
                );
        std::vector<vtkSmartPointer<vtkActor>> getGeometryActors();
        void addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer);
        bool setContactNormal(const Eigen::Matrix4f& contact_normal);
        void setPointColor(const std::array<unsigned char, 4>& color);
        void setForceColor(const std::array<unsigned char, 4>& color);
        void setConeColor(const std::array<unsigned char, 4>& color);
    };
}

#endif