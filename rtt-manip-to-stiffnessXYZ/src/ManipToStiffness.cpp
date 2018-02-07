#include "ManipToStiffness.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ManipToStiffnessXYZ::ManipToStiffnessXYZ(std::string const& name) : TaskContext(name){
    initializePorts();

    stiffness_max_trans = 5000;
    stiffness_max_rot = 300;// ToDo: come up with reasonable values
    stiffness_min = 10;
    scale_max = 0.9;
    scale_min = 0.1;



    //addProperty("manip_factor",manip_factor).doc("The manipulability factor for chaning stiffness");
    addProperty("stiffness_max_trans",stiffness_max_trans).doc("Max Stiffness translation for the linear heuristic");
    addProperty("stiffness_max_rot",stiffness_max_rot).doc("Max Stiffness for rotation for the linear heuristic");
    addProperty("stiffness_min",stiffness_min).doc("Min Stiffness for the linear heuristic");



}



bool ManipToStiffnessXYZ::configureHook(){
    return true;
}

bool ManipToStiffnessXYZ::startHook(){
    return true;
}

void ManipToStiffnessXYZ::updateHook(){

    manip_measure_in_flow = manip_measure_in_port.read(manip_measure_in_data);

    elipse_x = std::abs(manip_measure_in_data.data()[0]);
    elipse_y = std::abs(manip_measure_in_data.data()[1]);
    elipse_z = std::abs(manip_measure_in_data.data()[2]);

    //RTT::log(RTT::Critical) << "Elipse_length: ["<<elipse_x<<" , "<<elipse_y<<" , "<<elipse_z<<" , "<<"]" <<RTT::endlog();

    max_elipse = fmax(fmax(elipse_x,elipse_y),elipse_z);

    scaled_x = elipse_x/max_elipse;
    scaled_y = elipse_y/max_elipse;
    scaled_z = elipse_z/max_elipse;


    //RTT::log(RTT::Critical) << "Scales: ["<<scaled_x<<" , "<<scaled_y<<" , "<<scaled_z<<" , "<<"]" <<RTT::endlog();

    cart_stiffdamp_out_data.stiffness(0) = stiffness_max_trans*manip_calculate(scaled_x);
    cart_stiffdamp_out_data.stiffness(1) = stiffness_max_trans*manip_calculate(scaled_y);
    cart_stiffdamp_out_data.stiffness(2) = stiffness_max_trans*manip_calculate(scaled_z);


    cart_stiffdamp_out_data.stiffness(3) = stiffness_max_rot*manip_calculate(scaled_x);  // Or keep constant, discuss later with Pouya
    cart_stiffdamp_out_data.stiffness(4) = stiffness_max_rot*manip_calculate(scaled_y);
    cart_stiffdamp_out_data.stiffness(5) = stiffness_max_rot*manip_calculate(scaled_z);

    //RTT::log(RTT::Critical) <<cart_stiffdamp_out_data.stiffness(0) << " :Stiffness X ]  ["<<cart_stiffdamp_out_data.stiffness(1) << " :Stiffness Y ]  ["<<cart_stiffdamp_out_data.stiffness(2) << " :Stiffness Z ]  ["<<RTT::endlog();

    // Filter these output later
    cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);


}


double ManipToStiffnessXYZ::manip_calculate(double manip_factor){


    if(manip_factor < scale_min){
        scale_factor= 1;
    }

    else if(manip_factor > scale_max){
        scale_factor = 0.002;
    }

    else if(manip_factor <= scale_max && manip_factor >= scale_min)
    {

        scale_factor= 1.12475-(1.2475*manip_factor); // Slope intercpet, parameterize later

    }

    return scale_factor;



}

void ManipToStiffnessXYZ::stopHook() {
}

void ManipToStiffnessXYZ::cleanupHook() {
}

void ManipToStiffnessXYZ::initializePorts(){

    /** OUTPUT PORTS    **/

    cart_stiffdamp_out_data = rstrt::dynamics::JointImpedance(6);
    cart_stiffdamp_out_port.setName("cart_stiffdamp_out_port");
    cart_stiffdamp_out_port.setDataSample(cart_stiffdamp_out_data);
    ports()->addPort(cart_stiffdamp_out_port);

    /** INPUT PORTS **/

    manip_measure_in_flow = RTT::NoData;
    manip_measure_in_port.setName("manip_measure_in_port");
    manip_measure_in_data = Eigen::Vector3d();
    ports()->addPort(manip_measure_in_port);



}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ManipToStiffnessXYZ)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ManipToStiffnessXYZ)
