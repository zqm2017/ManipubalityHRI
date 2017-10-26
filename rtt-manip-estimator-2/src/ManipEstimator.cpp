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


    //TODO Add the tool to the urdf and load the correct urdf later


    addProperty("hand_base",hand_base)
            .doc("Name of the base joint of the human arm");
    addProperty("hand_eef",hand_eef)
            .doc("Name of the wrist joint or the holding position");
    addProperty("urdf_path",hand_urdf_path)
            .doc("Path of the hand urdf");



    // Parsing the arm urdf

    if(!hand_model.initFile(hand_urdf_path.c_str())){
        std::cout<<"ERROR"<<std::endl;
        RTT::log(RTT::Error)<<"Couldn't get model from URDF"<<RTT::endlog();
    }
    if (!kdl_parser::treeFromUrdfModel(hand_model, hand_tree)) {

        RTT::log(RTT::Error) << "Couldn't get tree from model"<<RTT::endlog();

    }

}



bool ManipEstimator::configureHook(){

    hand_tree.getChain(hand_base, hand_eef, hand_chain);


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

    cur_shoulder_pose_in_flow = cur_shoulder_pose_in_port.read(cur_shoulder_pose_in_data);  // Read the current shoulder position
    cur_grip_pose_in_flow = cur_grip_pose_in_port.read(cur_grip_pose_in_data);  // Read the current grip position
    // Kinematic chain should be fitted between these two poses
    // ToDo:  Get the default initial values for the experiment

    // Creating a static segment at the new shoulder position
    KDL::Segment("shoulder_base",KDL::Joint(KDL::Joint::None), cur_shoulder_pose_in_data);

    // Adding the hand tree to the segment named "shoulder_base"
    hand_tree_updated.addChain(hand_chain,"shoulder_base");
    hand_tree_updated.getChain("shoulder_base",hand_eef,hand_chain_updated);


    // Solving for the new chain
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(hand_chain_updated,1e-5,100,1e-15);
    ik_solver.CartToJnt(q_hand_init,cur_grip_pose_in_data,q_hand_out);

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
    cur_shoulder_pose_in_port.setName("cur_shoulder_pose_in_port");
    cur_shoulder_pose_in_data = KDL::Frame();
    ports()->addPort(cur_shoulder_pose_in_port);

    cur_grip_pose_in_flow = RTT::NoData;
    cur_grip_pose_in_port.setName("cur_grip_pose_in_port");
    cur_grip_pose_in_data = KDL::Frame();
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
ORO_CREATE_COMPONENT(ManipEstimator)
