#include "ManipEstimator.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <kdl/frameacc_io.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <rtt_roscomm/rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>

#define DOF_HAND 4  /**  The number of Degree of freedom for the human arm model**/
ManipEstimator::ManipEstimator(std::string const& name) : TaskContext(name){
    initializePorts();

    q_hand_init.resize(DOF_HAND);
    q_hand_out.resize(DOF_HAND);


    /** Default arm parameters **/
    hand_base = "left_shoulder_forward_link";
    hand_eef = "left_wrist_link";
    hand_urdf_path = "/home/sgo/git_repos/ManipubalityHRI/URDF_Generator/gen_arm.urdf";     //TODO Add the tool to the urdf  and load the correct urdf later

    addProperty("hand_base",hand_base)
            .doc("Name of the base joint of the human arm");
    addProperty("hand_eef",hand_eef)
            .doc("Name of the wrist joint or the holding position");
    addProperty("urdf_path",hand_urdf_path)
            .doc("Path of the hand urdf");


    /** Loading the human arm model and crearing a tree **/
    if(!hand_model.initFile(hand_urdf_path.c_str())){
        std::cout<<"check path:"<< hand_urdf_path <<std::endl;
        std::cout<<"ERROR"<<std::endl;
        RTT::log(RTT::Error)<<"Couldn't get hand model from URDF"<<RTT::endlog();
    }
    else
        RTT::log(RTT::Info)<<"Human Arm URDF loaded"<<RTT::endlog();
    if (!kdl_parser::treeFromUrdfModel(hand_model, hand_tree)) {
        RTT::log(RTT::Error) << "Couldn't get tree from model"<<RTT::endlog();
    }

}



bool ManipEstimator::configureHook(){

    //    hand_tree.getChain(hand_base, hand_eef, hand_chain);
    hand_tree.getChain("base_link", "left_wrist_link", hand_chain);

    hand_jac = KDL::Jacobian(DOF_HAND);

    /** Choosing an initial value for the IK solver **/
    q_hand_init.data(0) = 1.5;
    q_hand_init.data(1) = 1.5;
    q_hand_init.data(2) = 1.5;
    q_hand_init.data(3) = 1.5; // TODO: make guesses near to the real necessary values


    /** Creating output ports for visualization **/

    arm_conf_out_data.position.resize(DOF_HAND);
    arm_conf_out_data.name.resize(DOF_HAND);
    arm_conf_out_port.setName("arm_conf_out_port");
    arm_conf_out_port.setDataSample(arm_conf_out_data);
    this->ports()->addPort(arm_conf_out_port);

    manip_ros_out_port.setName("manip_ros_out_port");
    manip_ros_out_port.setDataSample(manip_ros_out_data);
    this->ports()->addPort(manip_ros_out_port);

    return true;
}

bool ManipEstimator::startHook(){
    /** Out stream for publishing the arm joint states to the ROS**/
    arm_conf_out_port.createStream(rtt_roscomm::topic("/joint_states"));
    manip_ros_out_port.createStream(rtt_roscomm::topic("/manipmeasure"));
    return true;
}

void ManipEstimator::updateHook(){


    cur_shoulder_pose_in_flow = cur_shoulder_pose_in_port.read(cur_shoulder_pose_in_data);  // Read the current shoulder position
    cur_grip_pose_in_flow = cur_grip_pose_in_port.read(cur_grip_pose_in_data);  // Read the current grip position
    // Kinematic chain should be fitted between these two poses

    /** Creating KDL frames from the pose read from input ports **/
    shoulder_pos.x(cur_shoulder_pose_in_data.position.x);
    shoulder_pos.y(cur_shoulder_pose_in_data.position.y);
    shoulder_pos.z(cur_shoulder_pose_in_data.position.z);
    shoulder_rot.Quaternion(cur_shoulder_pose_in_data.orientation.x,cur_shoulder_pose_in_data.orientation.y,cur_shoulder_pose_in_data.orientation.z,cur_shoulder_pose_in_data.orientation.w);

    grip_pos.x(cur_grip_pose_in_data.position.x);
    grip_pos.y(cur_grip_pose_in_data.position.y);
    grip_pos.z(cur_grip_pose_in_data.position.z);
    grip_rot.Quaternion(cur_grip_pose_in_data.orientation.x,cur_grip_pose_in_data.orientation.y,cur_grip_pose_in_data.orientation.z,cur_grip_pose_in_data.orientation.w);

    KDL::Frame shoulder_frame(shoulder_rot,shoulder_pos);

    KDL::Frame grip_frame(grip_rot,grip_pos);

    // Creating a static segment at the new shoulder position
    KDL::Segment("shoulder_base",KDL::Joint(KDL::Joint::None), shoulder_frame);


    /** Solving the IK for the chain with respect the new grip position **/
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(hand_chain,1e-5,100,1e-15);
    ik_solver.CartToJnt(q_hand_init,grip_frame,q_hand_out);


    q_hand_init = q_hand_out; // Updating the intial guess for the joint config so that we get a near solution to the existing joint config

    //std::cout<<q_hand_init.data.transpose()<<std::endl;

    arm_conf_out_data.name[0] = "left_shoulder_forward_joint";
    arm_conf_out_data.name[1] = "left_shoulder_up_joint";
    arm_conf_out_data.name[2] = "left_shoulder_side_joint";
    arm_conf_out_data.name[3] = "left_elbow_joint";
    for(int i = 0 ; i < DOF_HAND; ++i)
        arm_conf_out_data.position[i] = q_hand_init.data(i);

    /** Solve for the Jacobian **/

    KDL::ChainJntToJacSolver jnt_to_jac_solver = KDL::ChainJntToJacSolver(hand_chain);
    //jnt_to_jac_solver = new KDL::ChainJntToJacSolver(hand_chain_updated);
    jnt_to_jac_solver.JntToJac(q_hand_out,hand_jac);



    /**  SVD **/

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_r(hand_jac.data.block<3,DOF_HAND>(0,0),Eigen::ComputeThinU |  Eigen::ComputeThinV);


    me_axis = svd_r.matrixU() * svd_r.singularValues();

    //std::cout<<"Elipsoid--> x: "<<me_axis.data()[0]<<" y: "<<me_axis.data()[1]<<" z: "<<me_axis.data()[2]<<std::endl;

    manipulability = sqrt((hand_jac.data.block<3,DOF_HAND>(0,0)*hand_jac.data.block<3,DOF_HAND>(0,0).transpose()).determinant());


    //std::cout<<"Manip"<<manipulability<<std::endl;
    manip_elipse_out_port.write(me_axis);
    manip_measure_out_port.write(manipulability);

//RTT::log(RTT::Critical) << manipulability<< " :Manip"<<RTT::endlog();
    arm_conf_out_data.header.stamp = rtt_rosclock::host_now();
    arm_conf_out_port.write(arm_conf_out_data);

    manip_ros_out_port.write(manipulability);
}




void ManipEstimator::stopHook() {
}

void ManipEstimator::cleanupHook() {
}





void ManipEstimator::initializePorts(){

    /** OUTPUT PORTS    **/

    manip_elipse_out_data = Eigen::Vector3d();
    manip_elipse_out_port.setName("manip_elipse_out_port");
    manip_elipse_out_port.setDataSample(manip_elipse_out_data);
    ports()->addPort(manip_elipse_out_port);

    manip_measure_out_data = double();
    manip_measure_out_port.setName("manip_measure_out_port");
    manip_measure_out_port.setDataSample(manip_measure_out_data);
    ports()->addPort(manip_measure_out_port);

    /** INPUT PORTS **/
    cur_shoulder_pose_in_flow = RTT::NoData;
    cur_shoulder_pose_in_port.setName("cur_shoulder_pose_in_port");
    cur_shoulder_pose_in_data = geometry_msgs::Pose();
    ports()->addPort(cur_shoulder_pose_in_port);

    cur_grip_pose_in_flow = RTT::NoData;
    cur_grip_pose_in_port.setName("cur_grip_pose_in_port");
    cur_grip_pose_in_data = geometry_msgs::Pose();
    ports()->addPort(cur_grip_pose_in_port);



}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ManipEstimator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//ORO_CREATE_COMPONENT_TYPE()
ORO_CREATE_COMPONENT(ManipEstimator)
