/*
 * Show a simple scene with an object, a hand and two contact
 * points. This example has everything this library offers.
*/

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

    // Set up the object, its texture, pose and opacity

    Eigen::Matrix4f object_pose = Eigen::Matrix4f::Identity();
    object_pose.col(3) << 0.2, 0.2, 0.0, 1.0;
    manip_env.addObject(object_path, object_texture, object_pose, 0.2);

    // Spawn in a conveniently placed gripper, with proper joint values

    std::vector<float> joint_values = {0.04, 0.04};
    Eigen::Matrix4f root_pose = Eigen::Matrix4f::Identity();
    root_pose.col(3) << 0.2, 0.2, 0.17, 1;
    root_pose.block<3,3>(0,0) <<  0,  1,  0,
                                  1,  0,  0,
                                  0,  0,  -1;
    manip_env.addHand(urdf_filename, root_pose, joint_values);

    // Show two contact points with the normalized force and friction cone

    Eigen::Matrix4f contact_1 = Eigen::Matrix4f::Identity();
    contact_1.block<3,3>(0,0) <<  0,  0,  1,
                                  0,  1,  0,
                                  -1,  0,  0;
    contact_1.col(3) << 0.16, 0.2, 0.07, 1.0;
    manip_env.addContactPoint(contact_1,
                              0.4,
                              true, true, true);

    Eigen::Matrix4f contact_2 = Eigen::Matrix4f::Identity();
    contact_2.block<3,3>(0,0) <<  0,  0, -1,
                                  0,  1,  0,
                                  1,  0,  0;
    contact_2.col(3) << 0.24, 0.2, 0.07, 1.0;
    manip_env.addContactPoint(contact_2,
                              0.4,
                              true, true, true);

    // Render scene

    manip_env.render();

    return EXIT_SUCCESS;
}