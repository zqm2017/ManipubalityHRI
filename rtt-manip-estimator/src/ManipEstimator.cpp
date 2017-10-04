#include "ManipEstimator.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ManipEstimator::ManipEstimator(std::string const& name) : TaskContext(name){
    initializePorts();
    q.resize(DOF_SIZE);
    q_hand_init.resize(HAND_DOF);
    q_hand_out.resize(HAND_DOF);

    //me_axis= {0,0,0};

    hand_base = "left_shoulder_forward_link";
    hand_eef = "left_hand_link";
    hand_urdf_path = "/home/sgo/git-repos/ManipulabilityHRI/hand.urdf";

    robot_base = "lwr_arm_base_link";
    robot_eef = "lwr_arm_7_link";
    robot_urdf_path = "/home/sgo/citk/systems/cogimon-minimal-nightly/share/gazebo/models/cogimon/kuka-lwr-4plus/model.urdf";

    //TODO Add the tool to the urdf and load the correct urdf later


    addProperty("hand_base",hand_base)
            .doc("Name of the base joint of the human arm");
    addProperty("hand_eef",hand_eef)
            .doc("Name of the wrist joint or the holding position");
    addProperty("urdf_path",hand_urdf_path)
            .doc("Path of the hand urdf");


    addProperty("robot_base",robot_base)
            .doc("Name of the base joint of the human arm");
    addProperty("robot_eef",robot_eef)
            .doc("Name of the wrist joint or the holding position");
    addProperty("robot_urdf_path",robot_urdf_path)
            .doc("Path of the hand urdf");


    // Parsing the arm urdf

    if(!hand_model.initFile(hand_urdf_path.c_str())){
        std::cout<<"ERROR"<<std::endl;
        RTT::log(RTT::Error)<<"Couldn't get model from URDF"<<RTT::endlog();
    }
    if (!kdl_parser::treeFromUrdfModel(hand_model, hand_tree)) {

        RTT::log(RTT::Error) << "Couldn't get tree from model"<<RTT::endlog();

    }

    // Parsing the robot urdf

    if(!robot_model.initFile(robot_urdf_path.c_str())){
        std::cout<<"ERROR"<<std::endl;
        RTT::log(RTT::Error)<<"Couldn't get model from robot URDF"<<RTT::endlog();
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {


        RTT::log(RTT::Error) << "Couldn't get tree from model"<<RTT::endlog();

    }

}



bool ManipEstimator::configureHook(){

    hand_tree.getChain(hand_base, hand_eef, hand_chain);

    robot_tree.getChain(robot_base, robot_eef, robot_chain);

    fk_solver = new KDL::ChainFkSolverPos_recursive(robot_chain);

    //q_hand_init.data() = {0,0,0,0,0};  // TODO, check and make sensible quess
    q_hand_init.data(0) = 0;
    q_hand_init.data(1) = 0;
    q_hand_init.data(2) = 0;
    q_hand_init.data(3) = 0;
    q_hand_init.data(4) = 0;

}

bool ManipEstimator::startHook(){
    return true;
}

void ManipEstimator::updateHook(){

    cur_base_pose_in_flow = cur_base_pose_in_port.read(cur_base_pose_in_data); // Reading the current pose of the robot base

    cur_joint_angles_in_flow = cur_joint_angles_in_port.read(cur_joint_angles_in_data); // Reading the current joint angles of the robot

    // Doing the forward Kinematics for the KUKA LWR

    for(int i = 0; i < DOF_SIZE; i++){
        q(i) = cur_joint_angles_in_data.angles.data()[i];
    }
    //q.data=cur_joint_angles_in_data.angles;

    fk_solver->JntToCart(q,cart_pose_out); // Solves the Forward kinematics of the robot with respect to the current joint config


    /** New Cartesian tool fram wrt camera fram is cur_base_pose_in_data * cart_pose_out **/
    cur_target_cart_pose = cart_pose_out * cur_base_pose_in_data; // This is the target pos for the human hand

    cur_shoulder_pose_in_flow = cur_shoulder_pose_in_port.read(cur_shoulder_pose_in_data); // // Read the current shoulder position

    // Creating a static segment at the new shoulder position
    KDL::Segment("shoulder_base",KDL::Joint(KDL::Joint::None), cur_base_pose_in_data);

    // Adding the hand tree to the segment named "shoulder_base"
    hand_tree_updated.addChain(hand_chain,"shoulder_base");
    hand_tree_updated.getChain("shoulder_base",hand_eef,hand_chain_updated);


    // Solving for the new chain
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(hand_chain_updated,1e-5,100,1e-15);
    ik_solver.CartToJnt(q_hand_init,cur_target_cart_pose,q_hand_out);

    q_hand_init = q_hand_out;

    /** Solve for the Jacobian **/

    jnt_to_jac_solver = new KDL::ChainJntToJacSolver(hand_chain_updated);
    jnt_to_jac_solver->JntToJac(q_hand_out,hand_jac);


    /**  SVD **/

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_r(hand_jac.data.block<3,7>(0,0),Eigen::ComputeThinU |  Eigen::ComputeThinV);


    me_axis = svd_r.matrixU() * svd_r.singularValues();


    manipulability = sqrt((hand_jac.data.block<3,7>(0,0)*hand_jac.data.block<3,7>(0,0).transpose()).determinant());


    manip_elipse_out_port.write(me_axis);
    manip_measure_out_port.write(manipulability);

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

    cur_shoulder_pose_in_flow = RTT::NoData;
    cur_shoulder_pose_in_port.setName("cur_base_pose_in_port");
    cur_shoulder_pose_in_data = KDL::Frame();
    ports()->addPort(cur_shoulder_pose_in_port);

    cur_base_pose_in_flow = RTT::NoData;
    cur_base_pose_in_port.setName("cur_base_pose_in_port");
    cur_base_pose_in_data = KDL::Frame();
    ports()->addPort(cur_base_pose_in_port);

    cur_joint_angles_in_flow = RTT::NoData;
    cur_joint_angles_in_port.setName("cur_joint_angles_in_port");
    cur_joint_angles_in_data = rstrt::kinematics::JointAngles(DOF_SIZE);
    ports()->addPort(cur_joint_angles_in_port);




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
ORO_CREATE_COMPONENT(ManipEstimator)
