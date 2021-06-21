#include <manip-env-visu/gripper.h>

namespace mev
{
    Gripper::Gripper()
    {
        gripper_name = "default_gripper";
        gripper_root_pose = Eigen::Matrix4f::Identity();
        gripper_root_link = nullptr;
        gripper_joints.clear();
    }

    Gripper::Gripper(const urdf::ModelInterfaceSharedPtr source_urdf) : Gripper()
    {
        initGripperFromURDF(source_urdf);
    }

    Gripper::Gripper(const std::string& gripper_urdf_filename) : Gripper()
    {
        initGripperFromURDF(gripper_urdf_filename);
    }

    void Gripper::initGripperFromURDF(const urdf::ModelInterfaceSharedPtr source_urdf)
    {
        urdf_model = source_urdf;
        gripper_name = urdf_model->getName();
        gripper_root_link = std::make_shared<Link> (source_urdf->getRoot(), nullptr, urdf_path);
        // follow the tree recursively
        initLinkTree(gripper_root_link);
    }

    void Gripper::initGripperFromURDF(const std::string& gripper_urdf_filename)
    {
        if (!fileExists(gripper_urdf_filename))
        {
            std::cout << "There is no such URDF file" << std::endl;
            return;
        }
        urdf_model = urdf::parseURDFFile(gripper_urdf_filename);
        if (urdf_model)
        {
            std::cout << "URDF file parsed successfully" << std::endl;
            urdf_path = gripper_urdf_filename;
            initGripperFromURDF(urdf_model);
        }
        else
        {
            std::cout << "URDF file failed to parse" << std::endl;
        }
    }

    void Gripper::initLinkTree(std::shared_ptr<Link> root_link)
    {
        // for every child of this link
        // call this function
        for (auto urdf_child_link : root_link->getURDFLink()->child_links)
        {
            std::shared_ptr<Link> child_link = std::make_shared<Link> (urdf_child_link, root_link, urdf_path);
            root_link->addChildLink(child_link);
            gripper_joints.push_back(child_link->getParentToLinkJoint());
            initLinkTree(child_link);
        }
        return;
    }

    void Gripper::refreshGripperGeometries()
    {
        // here, we call the visual geometry setGeometryWorldPose function with respect to the root pose of the gripper
        // for every link
        refreshGripperGeometries(gripper_root_link, gripper_root_pose);
    }

    void Gripper::refreshGripperGeometries(std::shared_ptr<mev::Link> link, const Eigen::Matrix4f& link_tree_root_frame)
    {
        link->setLinkGeometryWorldPose(link_tree_root_frame * link->getAbsoluteLinkTransform());
        for (auto child : link->getLinkChildren())
        {
            refreshGripperGeometries(child, link_tree_root_frame);
        }
    }

    void Gripper::setGripperRootPose(const Eigen::Matrix4f& root_pose)
    {
        gripper_root_pose = root_pose;
        refreshGripperGeometries();
    }

    void Gripper::addGripperGeometriesToRenderer(std::shared_ptr<mev::Link> link, const vtkSmartPointer<vtkRenderer> renderer)
    {
        std::cout << "[DEBUG] adding geometry for link " << link->getLinkName() << std::endl;
        link->addGeometryToRenderer(renderer);
        // For each link, get the overall transform and set the geometry position
        for (auto child : link->getLinkChildren())
        {
            addGripperGeometriesToRenderer(child, renderer);
        }
    }

    void Gripper::addGripperGeometriesToRenderer(const vtkSmartPointer<vtkRenderer> renderer)
    {
        addGripperGeometriesToRenderer(gripper_root_link, renderer);
        refreshGripperGeometries();
    }

    void Gripper::setJointValues(const std::vector<float>& joint_values)
    {
        if (joint_values.size() != gripper_joints.size())
        {
            std::cout << "[ERROR] number of joint values different from number of gripper joints" << std::endl;
        }
        for (int joint_idx = 0; joint_idx < gripper_joints.size(); joint_idx++)
        {
            std::cout << "[DEBUG] setting value of joint " << gripper_joints[joint_idx]->getJointName() << " to " << joint_values[joint_idx] << std::endl;
            gripper_joints[joint_idx]->setJointValue(joint_values[joint_idx]);
        }
        refreshGripperGeometries();
    }

}