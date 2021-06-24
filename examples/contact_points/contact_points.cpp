/*
 * Generate a bunch of random contact points and visualizations.
 * Contact normals and what type of visualization for each contact
 * are randomized.
*/

#include <manip-env-visu/manipulation_env.h>
#include <Eigen/Core>
#include <math.h>
#include <random>

int main(int argc, char const *argv[])
{

    mev::ManipulationEnv manip_env;

    // Random generator of true and false booleans

    auto gen_bool = std::bind(std::uniform_int_distribution<>(0,1), std::default_random_engine());
    auto gen_friction = std::bind(std::uniform_real_distribution<>(0.1, 0.5), std::default_random_engine());
    auto gen_coord = std::bind(std::uniform_real_distribution<>(0.0, 0.2), std::default_random_engine());

    for (size_t idx = 0; idx < 10; ++idx)
    {
        Eigen::Affine3f t1;
        t1.setIdentity();
        Eigen::Vector3f rotation_1;
        rotation_1.setRandom();
        rotation_1.normalize();
        Eigen::Vector3f translation_1;
        translation_1 << static_cast<float>(gen_coord()), static_cast<float>(gen_coord()), static_cast<float>(gen_coord());
        t1 = t1 * Eigen::AngleAxisf(M_PI/2, rotation_1) * Eigen::Translation3f(translation_1);

        manip_env.addContactPoint(t1.matrix(),
                                static_cast<float>(gen_friction()),
                                gen_bool(),
                                gen_bool(),
                                gen_bool());
    }

    manip_env.render();

    return EXIT_SUCCESS;
}