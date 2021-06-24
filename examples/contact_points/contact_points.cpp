#include <manip-env-visu/manipulation_env.h>
#include <Eigen/Core>

int main(int argc, char const *argv[])
{
    mev::ManipulationEnv manip_env;

    manip_env.addContactPoint(Eigen::Matrix4f::Identity(),
                              0.4,
                              true,
                              true,
                              true);

    manip_env.render();

    return EXIT_SUCCESS;
}