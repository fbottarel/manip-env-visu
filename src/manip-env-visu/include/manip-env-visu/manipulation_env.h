#ifndef MANIP_ENV_H
#define MANIP_ENV_H

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <stdio.h>
#include <filesystem>

#include <manip-env-visu/gripper.h>
#include <manip-env-visu/contact.h>

namespace mev
{
    class ManipulationEnv
    {
    private:
        std::string background_color;
        vtkNew<vtkNamedColors> colors;
        std::vector<vtkSmartPointer<vtkAxesActor>> axes_list;
        vtkSmartPointer<vtkRenderer> renderer;
        vtkSmartPointer<vtkRenderWindow> render_window;
        vtkSmartPointer<vtkRenderWindowInteractor> window_interactor;
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> window_interactor_style;
    public:
        std::vector<std::shared_ptr<mev::Gripper>> hands_list;
        std::vector<std::shared_ptr<mev::VisualGeometry>> manipulation_objects;
        std::vector<std::shared_ptr<mev::Contact>> contacts;

        ManipulationEnv();
        ~ManipulationEnv();
        void addAxes(const Eigen::Matrix4f& transform,
                     const std::string& x_text,
                     const std::string& y_text,
                     const std::string& z_text);
        bool addHand(const std::string& gripper_urdf_filename,
                     const Eigen::Matrix4f& pose,
                     const std::vector<float> joint_values);
        bool addObject(const std::string& model_filename,
                       const std::string& texture_filename,
                       const Eigen::Matrix4f& pose,
                       const float& opacity = 1.0);
        bool addObject(const std::string& model_filename,
                       const Eigen::Matrix4f& pose,
                       const float& opacity = 1.0);
        void addContact(const Eigen::Matrix4f& contact_normal,
                             float friction_coeff = 0.0,
                             float force_magnitude = 1.0,
                             bool display_cone = false,
                             bool display_force = false,
                             bool display_contact_point = true);
        void addContactPoint(const Eigen::Vector3f& contact_origin,
                             std::array<unsigned char, 4> color = {0, 0, 255, 100});
        void addFrictionCone(const Eigen::Matrix4f& contact_normal,
                             float friction_coeff = 0.0,
                             float force_magnitude = 1.0,
                             bool display_normal = false,
                             bool display_contact_point = true,
                             std::array<unsigned char, 4> color = {0, 255, 0, 100});
        void addContactForce(const Eigen::Matrix4f& force_direction,
                             float force_magnitude = 1.0,
                             std::array<unsigned char, 4> color = {255, 255, 0, 100});
        void render();
    };
}

#endif