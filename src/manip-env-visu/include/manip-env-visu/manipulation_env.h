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
    /**
     * @brief Class representing the manipulation environment
     *
     * This class contains all the elements that need to get visualized and
     * methods to add new elements to the environment
     */
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
        /**
         * @brief Adds a reference frame to the environment
         *
         * @param transform The transformation with respect to the root
         * reference frame, as a 4x4 matrix
         * @param x_text Text to be displayed on the x axis
         * @param y_text Text to be displayed on the y axis
         * @param z_text Text to be displayed on the z axis
         */
        void addAxes(const Eigen::Matrix4f& transform,
                     const std::string& x_text,
                     const std::string& y_text,
                     const std::string& z_text);
        /**
         * @brief Adds a hand visualization to the environment
         *
         * @param gripper_urdf_filename The path to the URDF of the hand
         * @param pose The pose of the root node of the hand, in the world
         * reference frame
         * @param joint_values Values for each joint in the URDF
         * @return true If URDF parses right and the pose is ok
         * @return false If something went wrong
         */
        bool addHand(const std::string& gripper_urdf_filename,
                     const Eigen::Matrix4f& pose,
                     const std::vector<float> joint_values);
        /**
         * @brief Adds an object mesh from file
         *
         * @param model_filename Path to the mesh filename
         * @param texture_filename Path to the object texture, if available.
         * Otherwise, the object will have a default color
         * @param pose Pose of the object with respect to the world reference
         * frame
         * @param opacity Transparency of the object once visualized. 0.0 means
         * transparent, 1.0 means opaque
         * @return true If the filenames are ok and the pose matrix is ok
         * @return false Otherwise
         */
        bool addObject(const std::string& model_filename,
                       const std::string& texture_filename,
                       const Eigen::Matrix4f& pose,
                       const float& opacity = 1.0);
        /**
         * @brief Adds an object mesh from file
         *
         * @param model_filename Path to the mesh filename. A default color will
         * be attributed
         * @param pose Pose of the object with respect to the world reference
         * frame
         * @param opacity Transparency of the object once visualized. 0.0 means
         * transparent, 1.0 means opaque
         * @return true If the filenames are ok and the pose matrix is ok
         * @return false Otherwise
         */
        bool addObject(const std::string& model_filename,
                       const Eigen::Matrix4f& pose,
                       const float& opacity = 1.0);
        /**
         * @brief Generic method to add a contact to the visualization
         *
         * The contact reference frame has the Z axis coincident with the
         * contact normal
         *
         * @param contact_normal Transformation between world and contact
         * @param friction_coeff Friction coefficient [0,1]
         * @param force_magnitude Magnitude of the forces to display. These will
         * be scaled uniformly
         * @param display_cone Whether to display the friction cone
         * @param display_force Whether to display the normal force
         * @param display_contact_point Whether to display the contact point
         */
        void addContact(const Eigen::Matrix4f& contact_normal,
                             float friction_coeff = 0.0,
                             float force_magnitude = 1.0,
                             bool display_cone = false,
                             bool display_force = false,
                             bool display_contact_point = true);
        /**
         * @brief Method to add a contact point to the visualization
         *
         * @param contact_origin Coordinates of the point in the world reference
         * frame
         * @param color Color, in RGBA format
         */
        void addContactPoint(const Eigen::Vector3f& contact_origin,
                             std::array<unsigned char, 4> color = {0, 0, 255, 100});
        /**
         * @brief Method to add a friction cone to the visualization
         *
         * The contact reference frame has the Z axis coincident with the
         * contact normal
         *
         * @param contact_normal Transformation between world and contact
         * reference frame
         * @param friction_coeff Friction coefficient [0,1]
         * @param force_magnitude Magnitude of the forces to display. These will
         * be scaled uniformly
         * @param display_normal Whether to display the normal force
         * @param display_contact_point Whether to display the contact point
         * @param color Color, in RGBA format
         */
        void addFrictionCone(const Eigen::Matrix4f& contact_normal,
                             float friction_coeff = 0.0,
                             float force_magnitude = 1.0,
                             bool display_normal = false,
                             bool display_contact_point = true,
                             std::array<unsigned char, 4> color = {0, 255, 0, 100});
        /**
         * @brief Method to add a contact force to the visualization
         *
         * The reference frame has the Z axis coincident with the
         * force direction
         *
         * @param contact_normal Transformation between world and force reference frame
         * @param force_magnitude Magnitude of the forces to display. These will
         * be scaled uniformly
         * @param color Color, in RGBA format
         */
        void addContactForce(const Eigen::Matrix4f& force_direction,
                             float force_magnitude = 1.0,
                             std::array<unsigned char, 4> color = {255, 255, 0, 100});
        /**
         * @brief Spawn an interactive rendering window for the visualization
         *
         * Use the standard VTK camera control keybindings. Exit with [Q]
         */
        void render();
    };
}

#endif