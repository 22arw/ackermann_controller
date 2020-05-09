/**
 * @author Gérald Lelong
 */

#ifndef JOINT_H
#define JOINT_H

#include <hardware_interface/joint_command_interface.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <stdexcept>
//#include <iostream>   //Necessary to perform debugging

namespace ackermann_controller {

struct JointBase
{
    JointBase(
        const std::string& name,
        const std::string& base_link_name,
        urdf::ModelInterfaceSharedPtr model
    ):
        name_(name)
    {
        urdf::Vector3 position;
        joint_ = model->getJoint(name_);
        urdf::JointConstSharedPtr joint = joint_;

        if (!joint)
        {
            throw std::runtime_error(name_ + " couldn't be retrieved from model description");
        }

        while (joint && joint->parent_link_name != base_link_name)
        {
            urdf::Pose transform = joint->parent_to_joint_origin_transform;
            position = transform.position + transform.rotation * position;

            urdf::LinkConstSharedPtr link = model->getLink(joint->parent_link_name);
            joint = link->parent_joint;
        }

        lateral_deviation_ = - position.y;  //Added negative sign on 04/10 because it was reversing joints
    }

    std::string name_;
    double lateral_deviation_;
    urdf::JointConstSharedPtr joint_;
    hardware_interface::JointStateHandle handle_;
};

struct Joint: public JointBase
{
    Joint(
        const std::string& name,
        const std::string& base_link_name,
        urdf::ModelInterfaceSharedPtr model,
        hardware_interface::JointStateHandle handle
    ):
        JointBase(name, base_link_name, model),
        handle_(handle)
    {}

    virtual double getPosition() const
    {
        return handle_.getPosition();
    }

    hardware_interface::JointStateHandle handle_;
};

struct ActuatedJoint: public JointBase
{
    ActuatedJoint(
        const std::string& name,
        const std::string& base_link_name,
        urdf::ModelInterfaceSharedPtr model,
        hardware_interface::JointHandle handle
    ):
        JointBase(name, base_link_name, model),
        handle_(handle)
    {}

    virtual double getPosition() const
    {
        return handle_.getPosition();
    }

    void setCommand(double command)
    {
        handle_.setCommand(command);
    }

    hardware_interface::JointHandle handle_;
};

struct WheelBase
{
    WheelBase(
        const std::string& child_link_name,
        urdf::ModelInterfaceSharedPtr model)
    {
        urdf::LinkConstSharedPtr link = model->getLink(child_link_name);

        if (!link)
        {
            throw std::runtime_error("Link not found");
        }

        if (!link->collision)
        {
            throw std::runtime_error("Link " + link->name + " does not have collision description. Add collision description for link to urdf.");
        }

        if (!link->collision->geometry)
        {
            throw std::runtime_error("Link " + link->name + " does not have collision geometry description. Add collision geometry description for link to urdf.");
        }

        if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
        {
            throw std::runtime_error("Link " + link->name + " does not have cylinder geometry");
        }

        radius_ = (static_cast<urdf::Cylinder*>(link->collision->geometry.get()))->radius;
    }

    double radius_;
};

struct Wheel: public Joint, public WheelBase
{
    Wheel(
        const std::string& name,
        const std::string& base_link_name,
        urdf::ModelInterfaceSharedPtr model,
        hardware_interface::JointStateHandle handle
    ):
        Joint(name, base_link_name, model, handle),
        WheelBase(joint_->child_link_name, model), last_pos{new double(0)}
        {}

    double* last_pos;   //This was added to convert to position feedback, pointer was used to keep struct const
};

struct ActuatedWheel: public ActuatedJoint, public WheelBase
{
    ActuatedWheel(
        const std::string& name,
        const std::string& base_link_name,
        urdf::ModelInterfaceSharedPtr model,
        hardware_interface::JointHandle handle
    ):
        ActuatedJoint(name, base_link_name, model, handle),
        WheelBase(joint_->child_link_name, model)
    {}
};

}

#endif
