#ifndef JOINT_H
#define JOINT_H

#include <fstream>

#include "utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>

namespace mev
{
    class Link;

    enum JointType { JOINT_REVOLUTE,
                     JOINT_PRISMATIC,
                     JOINT_FIXED};

    class Joint
    {
        std::string joint_name;

        std::shared_ptr<mev::Link> parent_link;
        std::shared_ptr<mev::Link> child_link;

        mev::JointType type;

        float joint_value;
        Eigen::Matrix4f parent_to_joint_ref_frame;
        Eigen::Vector3f joint_axis;

        public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // mandatory when using eigen with fixed size matrices

        Joint();
        Joint(urdf::JointConstSharedPtr urdf_joint);
        void setLinks(std::shared_ptr<Link> parent_link, std::shared_ptr<Link> child_link);
        void setJointType(JointType type);
        void setJointAxis(const Eigen::Vector3f& joint_axis);
        void setJointRefFrame(const Eigen::Matrix4f& joint_reference_frame);
        void setJointValue(const float& joint_value);
        Eigen::Matrix4f getJointTransform();
        std::string getJointName();
    };
}
#endif