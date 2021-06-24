#include <manip-env-visu/manipulation_env.h>

namespace mev
{
    ManipulationEnv::ManipulationEnv()
    : background_color {"DarkSlateGray"}
    {
        renderer = vtkSmartPointer<vtkRenderer>::New();
        render_window = vtkSmartPointer<vtkRenderWindow>::New();
        window_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        window_interactor_style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

        renderer->SetBackground(colors->GetColor3d(background_color).GetData());
        render_window->SetSize(600, 600);
        render_window->AddRenderer(renderer);
        render_window->SetWindowName("Manipulation Environment");
        window_interactor->SetRenderWindow(render_window);
        window_interactor->SetInteractorStyle(window_interactor_style);

        axes_list.clear();
        hands_list.clear();
        manipulation_objects.clear();

        addAxes(Eigen::Matrix4f::Identity(), "x_world", "y_world", "z_world");
    }

    ManipulationEnv::~ManipulationEnv()
    {
    }

    void ManipulationEnv::addAxes(const Eigen::Matrix4f& transform,
                             const std::string& x_text,
                             const std::string& y_text,
                             const std::string& z_text)
    {
        vtkNew<vtkAxesActor> axes;
        vtkNew<vtkTransform> vtk_transform;
        vtkNew<vtkMatrix4x4> vtk_matrix;
        for (int idx=0; idx<transform.rows(); ++idx)
            for (int jdx=0; jdx<transform.cols(); ++jdx)
                vtk_matrix->SetElement(idx, jdx, transform(idx, jdx));
        vtk_transform->SetMatrix(vtk_matrix);
        axes->SetUserTransform(vtk_transform);
        axes->SetTotalLength(0.1,0.1,0.1);
        axes->GetXAxisCaptionActor2D ()->GetTextActor()->SetTextScaleModeToNone();
        axes->GetYAxisCaptionActor2D ()->GetTextActor()->SetTextScaleModeToNone();
        axes->GetZAxisCaptionActor2D ()->GetTextActor()->SetTextScaleModeToNone();
        axes->SetXAxisLabelText(x_text.c_str());
        axes->SetYAxisLabelText(y_text.c_str());
        axes->SetZAxisLabelText(z_text.c_str());
        axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
        axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
        axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(20);
        axes_list.push_back(axes);
        renderer->AddActor(axes);
    }

    bool ManipulationEnv::addHand(const std::string& gripper_urdf_filename, const Eigen::Matrix4f& pose, const std::vector<float> joint_values)
    {
        if (!std::filesystem::exists(gripper_urdf_filename))
        {
            std::cout << "[ERROR] gripper urdf file does not exist" << std::endl;
            return false;
        }
        std::shared_ptr<mev::Gripper> hand = std::make_shared<mev::Gripper> (gripper_urdf_filename);
        hand->setGripperRootPose(pose);
        hand->setJointValues(joint_values);
        hand->refreshGripperGeometries();
        hand->refreshGripperGeometries();
        hand->addGripperGeometriesToRenderer(renderer);
        hands_list.push_back(hand);
        return true;
    }

    bool ManipulationEnv::addObject(const std::string& model_filename, const std::string& texture_filename, const Eigen::Matrix4f& pose, const float& opacity)
    {
        if (!std::filesystem::exists(model_filename))
        {
            std::cout << "[ERROR] model file " << model_filename << " does not exist" << std::endl;
            return false;
        }
        std::shared_ptr<VisualGeometry> manip_object = std::make_shared<VisualGeometry> (model_filename, "SlateGray", texture_filename);
        manip_object->setGeometryWorldPose(pose);
        manip_object->setOpacity(opacity);
        manip_object->addGeometryToRenderer(renderer);
        manipulation_objects.push_back(manip_object);
        return true;
    }

    bool ManipulationEnv::addObject(const std::string& model_filename, const Eigen::Matrix4f& pose, const float& opacity)
    {
        if (!std::filesystem::exists(model_filename))
        {
            std::cout << "[ERROR] model file " << model_filename << " does not exist" << std::endl;
            return false;
        }
        std::shared_ptr<VisualGeometry> manip_object = std::make_shared<VisualGeometry> (model_filename, "SlateGray");
        manip_object->setGeometryWorldPose(pose);
        manip_object->setOpacity(opacity);
        manip_object->addGeometryToRenderer(renderer);
        manipulation_objects.push_back(manip_object);
        return true;
    }

    void ManipulationEnv::addContactPoint(const Eigen::Matrix4f& contact_normal,
                        float friction_coeff,
                        bool display_cone,
                        bool display_force,
                        bool display_contact_point
                        )
    {
        std::shared_ptr<mev::Contact> contact = std::make_shared<mev::Contact> (contact_normal,
                                                                                friction_coeff,
                                                                                display_cone,
                                                                                display_force,
                                                                                display_contact_point);
        contact->addGeometryToRenderer(renderer);
        contacts.push_back(contact);
    }

    void ManipulationEnv::render()
    {
        render_window->Render();
        window_interactor->Initialize();
        window_interactor->Start();
    }
}