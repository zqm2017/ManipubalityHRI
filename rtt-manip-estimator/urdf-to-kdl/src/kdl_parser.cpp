/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

/* Modified: Pouya Mohammadi
 * This is based on ROS stack and code by Wim, which I unROSed!
 * There are prbably some unused functions and there might be some errors in some cases.
 * Use this at your own risk!
 */

#include "kdl_parser.hpp"
#include "model.h"
#include <kdl/frames_io.hpp>
// for messages. Should be replaced with more approapriate printing mechanism
#include <iostream>
// #include <ros/console.h>

using namespace std;
using namespace KDL;

namespace kdl_parser{


// construct vector
Vector toKdl(urdf::Vector3 v)
{
    return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation toKdl(urdf::Rotation r)
{
    return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame toKdl(urdf::Pose p)
{
    return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint toKdl(boost::shared_ptr<urdf::Joint> jnt)
{
    Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

    switch (jnt->type){
    case urdf::Joint::FIXED:{
        return Joint(jnt->name, Joint::None);
    }
    case urdf::Joint::REVOLUTE:{
        Vector axis = toKdl(jnt->axis);
        return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
    }
    case urdf::Joint::CONTINUOUS:{
        Vector axis = toKdl(jnt->axis);
        return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::RotAxis);
    }
    case urdf::Joint::PRISMATIC:{
        Vector axis = toKdl(jnt->axis);
        return Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, Joint::TransAxis);
    }
    default:{
        std::cout<<"Converting unknown joint type of joint "<<jnt->name.c_str()<<" into a fixed joint."<<std::endl;
        // ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());

        return Joint(jnt->name, Joint::None);
    }
    }
    return Joint();
}

// construct inertia
RigidBodyInertia toKdl(boost::shared_ptr<urdf::Inertial> i)
{
    Frame origin = toKdl(i->origin);

    // the mass is frame indipendent
    double kdl_mass = i->mass;

    // kdl and urdf both specify the com position in the reference frame of the link
    Vector kdl_com = origin.p;

    // kdl specifies the inertia matrix in the reference frame of the link,
    // while the urdf specifies the inertia matrix in the inertia reference frame
    RotationalInertia urdf_inertia =
            RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);
    
    // Rotation operators are not defined for rotational inertia,
    // so we use the RigidBodyInertia operators (with com = 0) as a workaround
    RigidBodyInertia kdl_inertia_wrt_com_workaround =
            origin.M *RigidBodyInertia(0, Vector::Zero(), urdf_inertia);

    // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
    // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
    // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
    RotationalInertia kdl_inertia_wrt_com =
            kdl_inertia_wrt_com_workaround.getRotationalInertia();
    
    return RigidBodyInertia(kdl_mass,kdl_com,kdl_inertia_wrt_com);
}


// recursive function to walk through tree
bool addChildrenToTree(boost::shared_ptr<const urdf::Link> root, Tree& tree)
{
    std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
    std::cout<<"Link "<<root->name.c_str()<<" had "<<(int)children.size()<<" children."<<std::endl;
    // ROS_DEBUG("Link %s had %i children", root->name.c_str(), (int)children.size());

    // constructs the optional inertia
    RigidBodyInertia inert(0);
    if (root->inertial)
        inert = toKdl(root->inertial);

    // constructs the kdl joint
    Joint jnt = toKdl(root->parent_joint);

    // construct the kdl segment
    Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

    // add segment to tree
    tree.addSegment(sgm, root->parent_joint->parent_link_name);

    // recurslively add all children
    for (size_t i=0; i<children.size(); i++){
        if (!addChildrenToTree(children[i], tree))
            return false;
    }
    return true;
}


bool treeFromFile(const string& file, Tree& tree)
{
    TiXmlDocument urdf_xml;
    urdf_xml.LoadFile(file);
    return treeFromXml(&urdf_xml, tree);
}

// bool treeFromParam(const string& param, Tree& tree)
// {
//   urdf::Model robot_model;
//   if (!robot_model.initParam(param)){
//     std::cout<<"Could not generate robot model"<<std::endl;
//     // ROS_ERROR("Could not generate robot model");
//     return false;
//   }
//   return treeFromUrdfModel(robot_model, tree);
// }

bool treeFromString(const string& xml, Tree& tree)
{
    TiXmlDocument urdf_xml;
    urdf_xml.Parse(xml.c_str());
    return treeFromXml(&urdf_xml, tree);
}

bool treeFromXml(TiXmlDocument *xml_doc, Tree& tree)
{
    urdf::Model robot_model;
    if (!robot_model.initXml(xml_doc)){
        std::cout<<"Could not generate robot model"<<std::endl;
        // ROS_ERROR("Could not generate robot model");
        return false;
    }
    return treeFromUrdfModel(robot_model, tree);
}


bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
    tree = Tree(robot_model.getRoot()->name);

    // warn if root link has inertia. KDL does not support this
    if (robot_model.getRoot()->inertial)
        std::cout<<"The root link "<<robot_model.getRoot()->name.c_str()<<" has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF."<<std::endl;
    // ROS_WARN("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.", robot_model.getRoot()->name.c_str());

    //  add all children
    for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
        if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree))
            return false;

    return true;
}

}

