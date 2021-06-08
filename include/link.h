#ifndef LINK_H
#define LINK_H

#include <fstream>

#include "utils.h"
#include "visual_geometry.h"
#include "joint.h"

#include <Eigen/Core>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/link.h>

namespace mev
{
    class Link : public std::enable_shared_from_this<mev::Link>
    {
        std::string link_name;

        std::shared_ptr<mev::Link> parent_link;
        std::vector<std::shared_ptr<mev::Link>> children_link;

        Eigen::Matrix4f link_visual_origin;
        std::shared_ptr<mev::URDFVisualGeometry> link_visual_geometry;

        urdf::LinkConstSharedPtr urdf_link;
        std::string urdf_path; // path to urdf file

        std::shared_ptr<mev::Joint> parent_to_link_joint;

        public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // mandatory when using eigen with fixed size matrices

        Link(urdf::LinkConstSharedPtr urdf_link, std::shared_ptr<mev::Link> parent_link = nullptr, const std::string& urdf_path = "");
        std::string getLinkName();
        bool linkHasChildren();
        bool linkHasParent();
        bool linkHasGeometry();
        void setParentLink(std::shared_ptr<mev::Link> parent_link);
        void setParentJoint(std::shared_ptr<mev::Joint> parent_to_link_joint);
        void addChildLink(std::shared_ptr<mev::Link> child_link);
        void addGeometryToRenderer(vtkSmartPointer<vtkRenderer> renderer);
        void setLinkGeometryWorldPose(const Eigen::Matrix4f& pose);
        urdf::LinkConstSharedPtr getURDFLink();
        Eigen::Matrix4f getTransformationToParentRefFrame();
        Eigen::Matrix4f getAbsoluteLinkTransform();
        std::vector<std::shared_ptr<mev::Link>> getLinkChildren();
        std::shared_ptr<mev::Joint> getParentToLinkJoint();
    };
}
#endif