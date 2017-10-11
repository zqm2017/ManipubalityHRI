/*
 * Author:  Pouya Mohammadi
 * Date:    August 25, 2017
 * License: 
 * Description: 
 * 
 */

#ifndef RTTFRICARTIMPEDCTRL_HPP
#define RTTFRICARTIMPEDCTRL_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <kdl/frames.hpp>
#include <friremote_rt.h>
#include <fricomm_rt.h>

#define DOF_SIZE 7
#define CART_SIZE 6

class RttFriCartImpedCtrl : public RTT::TaskContext{
public:
	RttFriCartImpedCtrl(std::string const& name);
	bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();



private:
    //friRemote _fri;
    friRemote* _fri;
    std::string _host_ip, _server_ip;
    int _port;
    bool _recieved, _command;
    double time, old_time;

    // input ports
    RTT::InputPort<KDL::Frame> cartesian_pose_in_port;
    RTT::FlowStatus cartesian_pose_in_flow;
    KDL::Frame cartesian_pose_in_data;
//    std::vector<float> robot_data;
    float cart_data_robot[FRI_CART_FRM_DIM];

    RTT::InputPort<rstrt::dynamics::JointImpedance> stiff_damp_in_port;
    RTT::FlowStatus stiff_damp_in_flow;
    rstrt::dynamics::JointImpedance stiff_damp_in_data;

    RTT::InputPort<rstrt::dynamics::JointTorques> added_torques_in_port;
    RTT::FlowStatus added_torques_in_flow;
    rstrt::dynamics::JointTorques added_torques_in_data;

    RTT::InputPort<rstrt::kinematics::JointAngles> redundancy_resolution_in_port;
    RTT::FlowStatus redundancy_resolution_in_flow;
    rstrt::kinematics::JointAngles redundancy_resolution_in_data;

    float * torques;
    float * joints;

    // output ports
    RTT::OutputPort<KDL::Frame> cartesian_pose_out_port;
    KDL::Frame cartesian_pose_out_data;

    RTT::OutputPort<rstrt::kinematics::JointAngles> current_joint_values_out_port;
    rstrt::kinematics::JointAngles current_joint_values_out_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> current_joint_torques_out_port;
    rstrt::dynamics::JointTorques current_joint_torques_out_data;

    RTT::OutputPort<rstrt::dynamics::Wrench> current_cart_force_torque_out_port;
    rstrt::dynamics::Wrench current_cart_force_torque_out_data;

    // maybe add stiffness and damping
    // deffinitly add the currenc conf and currecnt torques


    void initPorts();
    void sense();
    bool getCommand();
    void move();
    void form_output_data();
    rstrt::geometry::Translation _ee_position;
    rstrt::geometry::Rotation _ee_orientation;
protected:

};
#endif // RTTFRICARTIMPEDCTRL_HPP
