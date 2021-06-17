#include <manip-env-visu/manipulation_env.h>

int main(int argc, char const *argv[])
{
    std::string urdf_filename = argv[1];
    std::string object_path = argv[2];
    std::string object_texture = argv[3];

    if (!fileExists(urdf_filename))
    {
        std::cout << "There is no such URDF file" << std::endl;
        return -1;
    }

    if (!fileExists(object_path))
    {
        std::cout << "There is no such mesh file" << std::endl;
        return -1;
    }

    mev::ManipulationEnv manip_env;

    Eigen::Matrix4f object_pose = Eigen::Matrix4f::Identity();
    object_pose.col(3) << 0.2, 0.2, 0.0, 1.0;
    manip_env.addObject(object_path, object_texture, object_pose);

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