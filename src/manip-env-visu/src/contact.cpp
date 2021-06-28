#include <manip-env-visu/contact.h>

namespace mev
{
    Contact::Contact(const Eigen::Matrix4f& contact_normal,
                     float friction_coeff,
                     float force_magnitude,
                     bool display_cone,
                     bool display_force,
                     bool display_contact_point)
    : friction_coeff {friction_coeff},
      force_magnitude{force_magnitude},
      display_cone {display_cone},
      display_force {display_force},
      display_contact_point {display_contact_point}
    {
        this->setContactNormal(contact_normal);

        this->setScaledForceMagnitude();

        // Create sources with specified dimensions
        friction_cone_source = vtkSmartPointer<vtkConeSource>::New();
        friction_cone_source->SetHeight(force_scaled_magnitude);
        friction_cone_source->SetRadius(force_scaled_magnitude * friction_coeff);
        friction_cone_source->SetResolution(100);
        friction_cone_source->SetCenter(0, 0, force_scaled_magnitude/2);
        friction_cone_source->SetDirection(0, 0, -1);

        // The arrow must lie on the z axis
        force_vector_source = vtkSmartPointer<vtkArrowSource>::New();
        force_vector_source->SetTipResolution(100);
        force_vector_source->SetShaftResolution(100);
        vtkNew<vtkTransform> transform;
        transform->Scale(force_norm, force_norm, force_scaled_magnitude); // Just to make the arrow thinner
        transform->RotateY(270);
        force_transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        force_transform_filter->SetTransform(transform);
        force_transform_filter->SetInputConnection(force_vector_source->GetOutputPort());

        point_source = vtkSmartPointer<vtkSphereSource>::New();
        point_source->SetRadius(point_size);
        point_source->SetPhiResolution(100);
        point_source->SetThetaResolution(100);

        // Create mappers and actors
        cone_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        cone_mapper->SetInputConnection(friction_cone_source->GetOutputPort());
        cone_actor = vtkSmartPointer<vtkActor>::New();
        cone_actor->SetMapper(cone_mapper);
        this->setConeColor(cone_color);
        VisualGeometry::setActorWorldPose(cone_actor, contact_normal);

        force_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        force_mapper->SetInputConnection(force_transform_filter->GetOutputPort());
        force_actor = vtkSmartPointer<vtkActor>::New();
        force_actor->SetMapper(force_mapper);
        this->setForceColor(force_color);
        VisualGeometry::setActorWorldPose(force_actor, contact_normal);

        point_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        point_mapper->SetInputConnection(point_source->GetOutputPort());
        point_actor = vtkSmartPointer<vtkActor>::New();
        point_actor->SetMapper(point_mapper);
        this->setPointColor(point_color);
        VisualGeometry::setActorWorldPose(point_actor, contact_normal);
    }

    bool Contact::setContactNormal(const Eigen::Matrix4f& contact_normal)
    {
        // Check that input matrix is a proper input
        if (contact_normal.row(3) != Eigen::RowVector4f(0.0, 0.0, 0.0, 1.0))
        {
            std::cout << "[WARNING] contact point normal matrix is not homogeneous" << std::endl;
            contact_normal_pose = Eigen::Matrix4f::Identity();
            return false;
        }
        else if (!isMatrixRotation(contact_normal.block<3,3>(0,0)))
        {
            std::cout << "[WARNING] contact point normal matrix rotation matrix is invalid" << std::endl;
            contact_normal_pose = Eigen::Matrix4f::Identity();
            return false;
        }
        else
        {
            contact_normal_pose = contact_normal;
            return true;
        }
    }

    void Contact::setActorColor(const vtkSmartPointer<vtkActor> actor, const std::array<unsigned char, 4>& color)
    {
        // VTK color components are specified as floats between 0 and 1
        std::array<double, 4> color_normalized;
        for (size_t idx = 0; idx < color.size(); idx++)
        {
            color_normalized[idx] = static_cast<float>(color[idx])/255;
        }
        actor->GetProperty()->SetColor(color_normalized[0],
                                       color_normalized[1],
                                       color_normalized[2]);
        actor->GetProperty()->SetOpacity(color_normalized[3]);
    }

    void Contact::setScaledForceMagnitude()
    {
        this->force_scaled_magnitude =  this->force_magnitude * this->force_norm;
    }

    void Contact::setPointColor(const std::array<unsigned char, 4>& color)
    {
        this->setActorColor(point_actor, color);
    }

    void Contact::setForceColor(const std::array<unsigned char, 4>& color)
    {
        this->setActorColor(force_actor, color);
    }

    void Contact::setConeColor(const std::array<unsigned char, 4>& color)
    {
        this->setActorColor(cone_actor, color);
    }

    void Contact::addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer)
    {
        if (display_contact_point)
            renderer->AddActor(point_actor);
        if (display_force)
            renderer->AddActor(force_actor);
        if (display_cone)
            renderer->AddActor(cone_actor);
        if (!(display_contact_point || display_force || display_cone))
            std::cout << "[WARNING] no actor was added for contact point" << std::endl;
    }

    void Contact::setForceMagnitude(const double& force_magnitude)
    {
        // For visualization purposes, forces need to be scaled down in magnitude
        // Otherwise, a 1N force would show up as a 1m arrow!
        this->force_magnitude = static_cast<float> (force_magnitude);
        setScaledForceMagnitude();
    }
}

