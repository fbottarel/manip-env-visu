#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <vtkPolyData.h>
#include <vtkSphereSource.h>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <stdio.h>

#include <manip-env-visu/manipulation_env.h>

#include <vtkPNGReader.h>
#include <vtkTexture.h>


int main(int argc, char const *argv[])
{
    std::string urdf_filename = argv[1];

    if (!fileExists(urdf_filename))
    {
        std::cout << "There is no such URDF file" << std::endl;
        return -1;
    }

    urdf::ModelInterfaceSharedPtr gripper_urdf_model = urdf::parseURDFFile(urdf_filename);

    if (gripper_urdf_model)
        std::cout << "URDF file parsed successfully" << std::endl;
    else
    {
        std::cout << "URDF file failed to parse" << std::endl;
        return -1;
    }

    mev::ManipulationEnv manip_env;

    Eigen::Matrix4f object_pose = Eigen::Matrix4f::Identity();
    object_pose.col(3) << 0.2, 0.2, 0.0, 1.0;
    manip_env.addObject("/home/fbottarel/workspace/manip-env-visu/models/objects/cracker_box/textured_simple.obj", "/home/fbottarel/workspace/manip-env-visu/models/objects/cracker_box/texture_map.png", object_pose);

    std::vector<float> joint_values = {0.04, 0.04};
    Eigen::Matrix4f root_pose = Eigen::Matrix4f::Identity();
    root_pose.col(3) << 0.2, 0.2, 0.17, 1;
    root_pose.block<3,3>(0,0) <<  0,  1,  0,
                                  1,  0,  0,
                                  0,  0,  -1;
    manip_env.addHand(urdf_filename, root_pose, joint_values);

    manip_env.render();

    return EXIT_SUCCESS;
}