#include "joint.h"

namespace mev
{
    Joint::Joint()
    {

    }

    Joint::Joint(urdf::JointConstSharedPtr urdf_joint)
    {
        joint_name = urdf_joint->name;

        switch (urdf_joint->type)
        {
        case urdf::Joint::PRISMATIC:
            type = JOINT_PRISMATIC;
            break;
        case urdf::Joint::REVOLUTE:
            type = JOINT_REVOLUTE;
            break;
        case urdf::Joint::FIXED:
            type = JOINT_FIXED;
            break;
        default:
            // otherwise, go for a fixed joint (for now)
            type = JOINT_FIXED;
            std::cout << "[WARNING] Joint " << joint_name << " type unknown, setting as FIXED";
            break;
        }

        joint_value = 0.0;

        parent_to_joint_ref_frame = getHomogeneousTransform(urdf_joint->parent_to_joint_origin_transform);
        joint_axis.setZero();
        // Joint axis will be processed accordingly to joint type
        joint_axis << urdf_joint->axis.x,
                      urdf_joint->axis.y,
                      urdf_joint->axis.z;
        joint_axis.normalize();
        setJointRefFrame(getHomogeneousTransform(urdf_joint->parent_to_joint_origin_transform));
        std::cout << "[DEBUG] created joint " << joint_name << std::endl;
    }

    void Joint::setLinks(std::shared_ptr<Link> parent_link, std::shared_ptr<Link> child_link)
    {
        parent_link = parent_link;
        child_link = child_link;
    }

    void Joint::setJointType(JointType type)
    {
        type = type;
    }

    void Joint::setJointAxis(const Eigen::Vector3f& joint_axis)
    {
        this->joint_axis = joint_axis;
        this->joint_axis.normalize();
    }

    void Joint::setJointRefFrame(const Eigen::Matrix4f& joint_reference_frame)
    {
        if (joint_reference_frame.block<3,3>(0,0).determinant() == 1.0 && joint_reference_frame(3,3) == 1)
            parent_to_joint_ref_frame = joint_reference_frame;
    }

    void Joint::setJointValue(const float& joint_value)
    {
        this->joint_value = joint_value;
    }

    Eigen::Matrix4f Joint::getJointTransform()
    {
        Eigen::Matrix4f transform;
        // Changes according to joint type
        switch (this->type)
        {
        case JOINT_FIXED:
            {
                transform = parent_to_joint_ref_frame;
                break;
            }
        case JOINT_PRISMATIC:
            {
                transform = Eigen::Matrix4f::Identity();
                transform.block<3,1>(0,3) = joint_value * joint_axis;
                transform = parent_to_joint_ref_frame * transform;
                break;
            }
        case JOINT_REVOLUTE:
            {
                transform = Eigen::Matrix4f::Identity();
                Eigen::AngleAxisf rotation_aa(joint_value, joint_axis);
                transform.block<3,3>(0,0) = rotation_aa.toRotationMatrix();
                transform = parent_to_joint_ref_frame * transform;
                break;
            }
        default:
            transform = Eigen::Matrix4f::Identity();
            break;
        }
        return transform;
    }

    std::string Joint::getJointName()
    {
        return joint_name;
    }
}
