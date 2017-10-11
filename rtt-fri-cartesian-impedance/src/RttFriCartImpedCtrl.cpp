/*
 * Author:  Pouya Mohammadi
 * Date:    August 25, 2017
 * License:
 * Description:
 *
 */

#include "RttFriCartImpedCtrl.hpp"
#include <rtt/Component.hpp>

RttFriCartImpedCtrl::RttFriCartImpedCtrl(std::string const& name) : TaskContext(name){
    _server_ip = "192.168.0.200";
    addProperty("server_ip", _server_ip).doc("IP of this machine");

    _host_ip = "192.168.0.21";
    addProperty("host_ip", _host_ip).doc("IP of robot");

    _port = 49938;
    addProperty("port", _port).doc("server port");

    initPorts();
}

void RttFriCartImpedCtrl::initPorts(){
    cartesian_pose_in_flow = RTT::NoData;
    cartesian_pose_in_port.setName("des_cart_pose");
    cartesian_pose_in_data = KDL::Frame();
    ports()->addPort(cartesian_pose_in_port);

    stiff_damp_in_flow = RTT::NoData;
    stiff_damp_in_port.setName("des_stiff_damp");
    stiff_damp_in_data = rstrt::dynamics::JointImpedance(DOF_SIZE);    
    ports()->addPort(stiff_damp_in_port);
    // set defualts for stiffness and damping. Try to make it more ellegant
    for (int i = 0; i < DOF_SIZE; ++i){
        stiff_damp_in_data.stiffness(i) = 400.0;
        stiff_damp_in_data.damping(i)   = 1.0;
    }

    added_torques_in_flow = RTT::NoData;
    added_torques_in_port.setName("des_added_torques");
    added_torques_in_data = rstrt::dynamics::JointTorques(DOF_SIZE);
    ports()->addPort(added_torques_in_port);

    redundancy_resolution_in_flow = RTT::NoData;
    redundancy_resolution_in_port.setName("des_null_space");
    redundancy_resolution_in_data = rstrt::kinematics::JointAngles(DOF_SIZE);
    ports()->addPort(redundancy_resolution_in_port);

    cartesian_pose_out_data = KDL::Frame();
    cartesian_pose_out_port.setName("current_cartesian_pose");
    cartesian_pose_out_port.setDataSample(cartesian_pose_out_data);
    ports()->addPort(cartesian_pose_out_port);

    current_joint_values_out_data = rstrt::kinematics::JointAngles(DOF_SIZE);
    current_joint_values_out_port.setName("current_joint_values");
    current_joint_values_out_port.setDataSample(current_joint_values_out_data);
    ports()->addPort(current_joint_values_out_port);

    current_joint_torques_out_data = rstrt::dynamics::JointTorques(DOF_SIZE);
    current_joint_torques_out_port.setName("current_joint_torques");
    current_joint_torques_out_port.setDataSample(current_joint_torques_out_data);
    ports()->addPort(current_joint_torques_out_port);

    current_cart_force_torque_out_data = rstrt::dynamics::Wrench();
    current_cart_force_torque_out_port.setName("current_cart_force_torque");
    current_cart_force_torque_out_port.setDataSample(current_cart_force_torque_out_data);
    ports()->addPort(current_cart_force_torque_out_port);

}

bool RttFriCartImpedCtrl::configureHook(){
    // TODO check the fourth argument here
    //_fri = friRemote(_port, _host_ip.c_str(), _server_ip.c_str(), this->getActivity()->thread()->getTask());
    _fri= new friRemote(_port, _host_ip.c_str(), _server_ip.c_str(), this->getActivity()->thread()->getTask());
    // TODO make a sense of this:
    char str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(_fri->remote.krcAddr.sin_addr), str, INET_ADDRSTRLEN);
    RTT::log(RTT::Info) << "Opening FRI Version "
                        << FRI_MAJOR_VERSION << "."
                        << FRI_SUB_VERSION << "."
                        << FRI_DATAGRAM_ID_CMD << "."
                        << FRI_DATAGRAM_ID_MSR
                        << " Interface for LWR ROS server"
                        << RTT::endlog();
    RTT::log(RTT::Info) << "Checking if the robot is Stopped..." << RTT::endlog();
    if(_fri->getState() == FRI_STATE_OFF ){
         RTT::log(RTT::Info) << "Please, start the KRL script now." << RTT::endlog();
    }
    return true;
}

bool RttFriCartImpedCtrl::startHook(){
    _fri->getCmdBuf().cmd.cmdFlags = 0;
    _fri->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTPOS;
    _fri->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTSTIFF;
    _fri->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTDAMP;
    _fri->getCmdBuf().cmd.cmdFlags |= FRI_CMD_TCPFT;
    _fri->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTPOS;
    _fri->setToKRLInt(14,20);  // where to get this value
    _fri->doDataExchange();
    return true;
}

void RttFriCartImpedCtrl::updateHook(){
//    time = RTT::os::TimeService::Instance()->getNSecs();
    _command = getCommand();
    sense();
    if(_command){
        move();
    }

    cartesian_pose_out_port.write(cartesian_pose_out_data);
    current_joint_values_out_port.write(current_joint_values_out_data);
    current_joint_torques_out_port.write(current_joint_torques_out_data);
    current_cart_force_torque_out_port.write(current_cart_force_torque_out_data);
}

bool RttFriCartImpedCtrl::getCommand(){
    if(!cartesian_pose_in_port.connected()){
        RTT::log(RTT::Warning) << "Cartesian pose port is not connected!" <<RTT::endlog();
        return false;
    } else {
        cartesian_pose_in_flow = cartesian_pose_in_port.readNewest(cartesian_pose_in_data);
        cart_data_robot[0]  = cartesian_pose_in_data.M.UnitX().x();
        cart_data_robot[1]  = cartesian_pose_in_data.M.UnitY().x();
        cart_data_robot[2]  = cartesian_pose_in_data.M.UnitZ().x();
        cart_data_robot[3]  = cartesian_pose_in_data.p.x();
        cart_data_robot[4]  = cartesian_pose_in_data.M.UnitX().y();
        cart_data_robot[5]  = cartesian_pose_in_data.M.UnitY().y();
        cart_data_robot[6]  = cartesian_pose_in_data.M.UnitZ().y();
        cart_data_robot[7]  = cartesian_pose_in_data.p.y();
        cart_data_robot[8]  = cartesian_pose_in_data.M.UnitX().z();
        cart_data_robot[9]  = cartesian_pose_in_data.M.UnitY().z();
        cart_data_robot[10] = cartesian_pose_in_data.M.UnitZ().z();
        cart_data_robot[11] = cartesian_pose_in_data.p.z();
    }

    if(stiff_damp_in_port.connected()){
        stiff_damp_in_flow = stiff_damp_in_port.readNewest(stiff_damp_in_data);
    }

    if(added_torques_in_port.connected()){
        added_torques_in_flow = added_torques_in_port.readNewest(added_torques_in_data);
        torques = added_torques_in_data.torques.data();
    } else {
        added_torques_in_flow = RTT::NoData;
        torques = NULL;
    }

    if(redundancy_resolution_in_port.connected()){
        redundancy_resolution_in_flow = redundancy_resolution_in_port.readNewest(redundancy_resolution_in_data);
        joints = redundancy_resolution_in_data.angles.data();
    } else {
        redundancy_resolution_in_flow = RTT::NoData;
        joints = NULL;
    }

    return true;
}

void RttFriCartImpedCtrl::move(){
    if(!_recieved){
        _fri->doSendData();
        return;
    }
    // not sure why. a practice noticed at josh's code
    if(_fri->getQuality()<2){
        RTT::log(RTT::Info) << _fri->doSendData()<< ":low qual sending"<<RTT::endlog();
        return;
    }

    _fri->doCartesianImpedanceControl(
                cart_data_robot,
                stiff_damp_in_data.stiffness.data(),
                stiff_damp_in_data.damping.data(),
                torques,
                joints,
                false);

    _fri->doSendData();
}

void RttFriCartImpedCtrl::stopHook() {
    // else if (_current_control_mode == ControlModes::JointTorqueCtrl){
    // std::vector<int> joint_scoped_names = getJointScopedNames();
    // for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
    //     _joint_trq(joint_scoped_names[i]) =0;
    //     zero_vector[joint_scoped_names[i]]=0.4;
    // }
    //     _fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(),
    //                 zero_vector, zero_vector, _joint_trq.data(), true);
    // }
    // int r = 0;
    // //while(_fri_inst->getFrmKRLInt(15) != 10 &&  (r>=0)){
    // while(r<20){
    //     _fri_inst->setToKRLInt(15, 20);
    //     _fri_inst->doDataExchange();
    //     r++;
    //     RTT::log(RTT::Critical) << r<< ":stopping"<<RTT::endlog();
    // }
    // _fri_inst->setToKRLInt(15, 0);
    // _fri_inst->doDataExchange();
    // _fri_inst->doDataExchange();
    // _fri_inst->doDataExchange();



    // Not sure about this. Seems trying to set one last value to kuka before shutting down
    // however, don't know why they repeate last line multiple times, as well as some other
    // part. Perhapse it's a matter of being super duper sure!
    _fri->doCartesianImpedanceControl(_fri->getMsrCartPosition()); // rest should be nullx4 and true

    int r = 0;
    while(r<20){
        _fri->setToKRLInt(15, 20);
        _fri->doDataExchange();
        r++;
        RTT::log(RTT::Critical) << r<< ":stopping"<<RTT::endlog();
    }
    _fri->setToKRLInt(15, 0);
    _fri->doDataExchange();
    _fri->doDataExchange();
    _fri->doDataExchange();
}

void RttFriCartImpedCtrl::cleanupHook() {

}

void RttFriCartImpedCtrl::sense(){
    // from joshua smith at https://github.com/smithjoshua001/rtt-lwr-hardware-integration
    _recieved = (_fri->doReceiveData()>=0);
    if(!_recieved){
        return;
    }

    _fri->setToKRLInt(15, 0);
    if(_fri->getQuality()<2){
        return;
    }

    if (_fri->getFrmKRLInt(15) < 10) {
        return;
    }

    if (_fri->getFrmKRLInt(15) == 10) {
        if(_fri->getQuality()>=2){
            _fri->setToKRLInt(15, 10);
            //_fri_inst->doDataExchange(); ??
        }
    }

    form_output_data();
}

void RttFriCartImpedCtrl::form_output_data(){
    for(int i = 0; i<DOF_SIZE; ++i){
        current_joint_values_out_data.angles(i) = _fri->getMsrCmdJntPosition()[i];
        current_joint_torques_out_data.torques(i) = _fri->getMsrJntTrq()[i];
    }

    // Getting the current external Force Torque reading
    current_cart_force_torque_out_data.forces(0) = _fri->getMsrEstExtForceTrq()[0];
    current_cart_force_torque_out_data.forces(1) = _fri->getMsrEstExtForceTrq()[1];
    current_cart_force_torque_out_data.forces(2) = _fri->getMsrEstExtForceTrq()[2];

    current_cart_force_torque_out_data.torques(0) = _fri->getMsrEstExtForceTrq()[3];
    current_cart_force_torque_out_data.torques(1) = _fri->getMsrEstExtForceTrq()[4];
    current_cart_force_torque_out_data.torques(2) = _fri->getMsrEstExtForceTrq()[5];


    // TODO check if getMsr** is populating its data once or on every call
    cartesian_pose_out_data.p.data[0] = _fri->getMsrCartPosition()[3];
    cartesian_pose_out_data.p.data[1] = _fri->getMsrCartPosition()[7];
    cartesian_pose_out_data.p.data[2] = _fri->getMsrCartPosition()[11];

    cartesian_pose_out_data.M.data[0] = _fri->getMsrCartPosition()[0];
    cartesian_pose_out_data.M.data[1] = _fri->getMsrCartPosition()[1];
    cartesian_pose_out_data.M.data[2] = _fri->getMsrCartPosition()[2];
    cartesian_pose_out_data.M.data[3] = _fri->getMsrCartPosition()[4];
    cartesian_pose_out_data.M.data[4] = _fri->getMsrCartPosition()[5];
    cartesian_pose_out_data.M.data[5] = _fri->getMsrCartPosition()[6];
    cartesian_pose_out_data.M.data[6] = _fri->getMsrCartPosition()[8];
    cartesian_pose_out_data.M.data[7] = _fri->getMsrCartPosition()[9];
    cartesian_pose_out_data.M.data[8] = _fri->getMsrCartPosition()[10];

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RttFriCartImpedCtrl)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(RttFriCartImpedCtrl)
