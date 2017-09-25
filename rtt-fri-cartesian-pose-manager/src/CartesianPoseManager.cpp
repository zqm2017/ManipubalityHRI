#include "CartesianPoseManager.hpp"
#include <rtt/Component.hpp>
#include <iostream>

CartesianPoseManager::CartesianPoseManager(std::string const& name) : TaskContext(name){
    initializePorts();
    task = QuinticPolynomial<float>();
    fin_pos = Eigen::Vector3f::Zero(3);
    task_set = false;
    stiffdamp_set = false;
    _stiffness_counter= 0;
    alpha=0.010;

    // TODO: make them get property
    //force_max = 15;
    //force_min = 2;
    //stiffness_max = 2200;
    //stiffness_min = 10;


    addOperation("setCartesianPose", &CartesianPoseManager::setCartPose, this, RTT::ClientThread)
            .doc("Set desiered cartesian pose.")
            .arg("position", "Desiered KDL::Position")
            .arg("orientation", "Desiered KDL::orientation")
            .arg("time", "Duration of task");

    addOperation("setRedundancyRes", &CartesianPoseManager::setRedRes, this, RTT::ClientThread)
            .doc("Set a joint configuration as redundancy resolution scheme for testing.")
            .arg("joint_vals","Desiered rstrt::kinematic::JointAngles configuration");

    addOperation("setCartesianStiffDamp", &CartesianPoseManager::setCartStiffDamp, this, RTT::ClientThread)
            .doc("Set a Cartesian Stiffness and Damping value for testing.")
            .arg("cart_stiffdamp","Desired cartesian damping and stiffness: 3 translational and 3 rotational");\

    // Todo : Add max min force property

}

bool CartesianPoseManager::setCartPose(KDL::Vector position, KDL::Rotation orientation, double duration){
    // -----------------------final-------------------------
    // orientation
    double x2,y2,z2,w2;
    orientation.GetQuaternion(x2,y2,z2,w2);
    fin_rot = Eigen::Quaternionf(w2,x2,y2,z2);
    fin_rot.normalize();
    // position
    fin_pos << position.x(), position.y(), position.z();
    // -----------------------initial-----------------------
    // orientation
    double x1,y1,z1,w1; // not reusing the above ones because reasons!
    cur_cart_pose_in_data.M.GetQuaternion(x1,y1,z1,w1);
    init_rot = Eigen::Quaternionf(w1,x1,y1,z1);
    // position
    init_pos << cur_cart_pose_in_data.p.x(), cur_cart_pose_in_data.p.y(),cur_cart_pose_in_data.p.z();

    // here orientaion would be done by slerp of Eigen and position by my quiuntic polynomial
    start_time = time;
    end_time   = start_time+duration;
    task.setParams(start_time, end_time, init_pos, fin_pos);
    task_set = true;
}

void CartesianPoseManager::setRedRes(rstrt::kinematics::JointAngles joint_vals){
    red_res_out_data = joint_vals;
}

void CartesianPoseManager::setCartStiffDamp(Eigen::VectorXf  cart_stiff, Eigen::VectorXf cart_damp){

    cart_stiffdamp_out_data.stiffness = cart_stiff;
    cart_stiffdamp_out_data.damping   = cart_damp;
    stiffdamp_set = true;
}

bool CartesianPoseManager::configureHook(){
    return true;
}

bool CartesianPoseManager::startHook(){
    return true;
}

void CartesianPoseManager::updateHook(){
    time = 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
    // read:
    cur_cart_pose_in_flow = cur_cart_pose_in_port.read(cur_cart_pose_in_data);
    des_cart_stiffdamp_in_flow = des_cart_stiffdamp_in_port.read(des_cart_stiffdamp_in_data);

    if(task_set){
        intermed_pos = task.getQ(time);
        // for readabilty:
        double tau = (time-start_time)/(end_time-start_time);
        double tt  = (6.0*std::pow(tau,5.0)-15.0*std::pow(tau,4.0)+10.0*std::pow(tau,3.0));
        intermed_rot = init_rot.slerp(tt,fin_rot);
        if (time >= end_time){
            task_set = false;
        }
        cart_pose_out_data.p.data[0] = intermed_pos(0);
        cart_pose_out_data.p.data[1] = intermed_pos(1);
        cart_pose_out_data.p.data[2] = intermed_pos(2);
        cart_pose_out_data.M = KDL::Rotation::Quaternion(intermed_rot.x(),intermed_rot.y(),intermed_rot.z(),intermed_rot.w());
        cart_pose_out_port.write(cart_pose_out_data);
    } else{
        // looping:
        // process:
        cart_pose_loop_out_data = cur_cart_pose_in_data;
        cart_pose_loop_out_port.write(cart_pose_loop_out_data);
    }


    if(stiffdamp_set){
        cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);
        stiffdamp_set = false;
    }
    else{
        cart_stiffdamp_out_port.write(des_cart_stiffdamp_in_data);
    }

    red_res_out_port.write(red_res_out_data);


    /*cur_force_torq_in_flow = cur_force_torq_in_port.read(cur_force_torq_in_data);

    resultant_force = sqrt(pow(cur_force_torq_in_data.forces.data()[0],2)+pow(cur_force_torq_in_data.forces.data()[1],2)+pow(cur_force_torq_in_data.forces.data()[2],2));


   stiffnessEstimator(resultant_force);

    for(int i = 0 ; i < 6 ; ++i){
        cart_stiffdamp_out_data.stiffness(i) = _filtered_stiffness;
        std::cout<<"Stiffness: "<<_filtered_stiffness<<std::endl;
    }*/


}


/*void CartesianPoseManager::stiffnessEstimator(double resultant_force)
{

    if(resultant_force < force_min){
        _stiffness = stiffness_max;
    }

    else if(resultant_force > force_max){
        _stiffness = stiffness_min;
    }

    else if(resultant_force <= force_max && resultant_force >= force_min)
    {

        _stiffness = ((stiffness_max-stiffness_min)/(force_min-force_max))*resultant_force+stiffness_max;

    }

    // Filtering using lowpass filter
    _stiffness_vector_in.push_back(_stiffness);
    //_stiffness_vector_in(_stiffness_counter) = _stiffness;
    _a = (1-alpha)/(1+alpha);
    _b = (1-_a)/2;
    _xml= 0.9;
    if(_stiffness_counter==0)
        _filtered_stiffness = _stiffness_vector_in[_stiffness_counter]+_xml;
    else
        _filtered_stiffness = _b*_stiffness_vector_in[_stiffness_counter]+ _b*_stiffness_vector_in[_stiffness_counter-1]+ _a*_stiffness_vector_out[_stiffness_counter-1];

    _stiffness_vector_out.push_back(_filtered_stiffness);
    //_stiffness_vector_out(_stiffness_counter) = _filtered_stiffness;
    _stiffness_counter++;
}
*/

void CartesianPoseManager::stopHook() {
}

void CartesianPoseManager::cleanupHook() {
}

void CartesianPoseManager::initializePorts(){

    /** OUTPUT PORTS    **/

    cart_stiffdamp_out_data = rstrt::dynamics::JointImpedance(6);
    cart_stiffdamp_out_port.setName("cart_stiffdamp_out_port");
    cart_stiffdamp_out_port.setDataSample(cart_stiffdamp_out_data);
    ports()->addPort(cart_stiffdamp_out_port);

    cart_pose_out_data = KDL::Frame();
    cart_pose_out_port.setName("cart_pose_out_port");
    cart_pose_out_port.setDataSample(cart_pose_out_data);
    ports()->addPort(cart_pose_out_port);

    cart_pose_loop_out_data = KDL::Frame();
    cart_pose_loop_out_port.setName("cart_pose_loop_out_port");
    cart_pose_loop_out_port.setDataSample(cart_pose_loop_out_data);
    ports()->addPort(cart_pose_loop_out_port);

    red_res_out_data = rstrt::kinematics::JointAngles(DOF_SIZE);
    red_res_out_port.setName("red_res_out_port");
    red_res_out_port.setDataSample(red_res_out_data);
    ports()->addPort(red_res_out_port);

    ext_trq_out_data = rstrt::dynamics::JointTorques(DOF_SIZE);
    ext_trq_out_port.setName("ext_trq_out_port");
    ext_trq_out_port.setDataSample(ext_trq_out_data);
    ports()->addPort(ext_trq_out_port);


    /** INPUT PORTS **/

    cur_cart_pose_in_flow = RTT::NoData;
    cur_cart_pose_in_port.setName("cur_cart_pose_in_port");
    cur_cart_pose_in_data = KDL::Frame();
    ports()->addPort(cur_cart_pose_in_port);

    cur_force_torq_in_flow = RTT::NoData;
    cur_force_torq_in_port.setName("cur_force_torque_in_port");
    cur_force_torq_in_data = rstrt::dynamics::Wrench();
    ports()->addPort(cur_force_torq_in_port);

    des_cart_stiffdamp_in_flow = RTT::NoData;
    des_cart_stiffdamp_in_port.setName("des_cart_stiffdamp_in_port");
    des_cart_stiffdamp_in_data = rstrt::dynamics::JointImpedance(6);
    ports()->addPort(des_cart_stiffdamp_in_port);


}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CartesianPoseManager)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(CartesianPoseManager)
