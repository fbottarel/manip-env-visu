/*
 * Generate a bunch of random contact points and visualizations.
 * Force magnitude, angle and color are randomized are randomized.
*/

#include <manip-env-visu/manipulation_env.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <random>

int main(int argc, char const *argv[])
{

    mev::ManipulationEnv manip_env;

    // Random generator of contacts
    auto gen_color = std::bind(std::uniform_int_distribution<>(0,255), std::default_random_engine());
    auto gen_angle = std::bind(std::uniform_real_distribution<>(0.0, 0.5), std::default_random_engine());
    auto gen_mag = std::bind(std::uniform_real_distribution<>(0.5, 5.0), std::default_random_engine());
    auto gen_friction = std::bind(std::uniform_real_distribution<>(0.1, 0.8), std::default_random_engine());
    auto gen_coord = std::bind(std::uniform_real_distribution<>(0.0, 0.5), std::default_random_engine());

    // Generate a random collection of contacts with varied colors
    for (size_t idx = 0; idx < 10; ++idx)
    {
        Eigen::Matrix4f base_contact_normal = Eigen::Matrix4f::Identity();
        base_contact_normal.block<3,1>(0,3) << gen_coord(), gen_coord(), gen_coord();

        Eigen::Matrix4f first_vector_transform(base_contact_normal);
        first_vector_transform.block<3,3>(0,0) = Eigen::AngleAxisf(gen_angle() * M_PI, Eigen::Vector3f::UnitY()).matrix();

        Eigen::Matrix4f second_vector_transform(base_contact_normal);
        second_vector_transform.block<3,3>(0,0) = Eigen::AngleAxisf(gen_angle() * M_PI, Eigen::Vector3f::UnitX()).matrix();

        manip_env.addFrictionCone(base_contact_normal, gen_friction(), 1.0, false, true);
        manip_env.addContactForce(first_vector_transform, gen_mag(), {static_cast<unsigned char> (gen_color()),
                                                                      static_cast<unsigned char> (gen_color()),
                                                                      static_cast<unsigned char> (gen_color()),
                                                                      255});
        manip_env.addContactForce(second_vector_transform, gen_mag(), {static_cast<unsigned char> (gen_color()),
                                                                       static_cast<unsigned char> (gen_color()),
                                                                       static_cast<unsigned char> (gen_color()),
                                                                       255});
    }

    manip_env.render();

    return EXIT_SUCCESS;
}