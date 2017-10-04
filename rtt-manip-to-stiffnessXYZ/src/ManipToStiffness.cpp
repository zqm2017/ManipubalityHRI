#include "ManipToStiffness.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ManipToStiffnessXYZ::ManipToStiffnessXYZ(std::string const& name) : TaskContext(name){
    initializePorts();

    stiffness_max = 2200; // ToDo: come up with reasonable values
    stiffness_min = 10;
    manip_max = 4;
    manip_min = 0.01;



    //addProperty("manip_factor",manip_factor).doc("The manipulability factor for chaning stiffness");
    addProperty("stiffness_max",stiffness_max).doc("Max Stiffness for the linear heuristic");
    addProperty("stiffness_min",stiffness_min).doc("Min Stiffness for the linear heuristic");
    addProperty("manip_max",manip_max).doc("Max manipulability expected");
    addProperty("manip_min",manip_min).doc("Min manipulability expected");


}



bool ManipToStiffnessXYZ::configureHook(){
    return true;
}

bool ManipToStiffnessXYZ::startHook(){
    return true;
}

void ManipToStiffnessXYZ::updateHook(){

    manip_measure_in_flow = manip_measure_in_port.read(manip_measure_in_data);

    elipse_x = manip_measure_in_data.data()[0];
    elipse_y = manip_measure_in_data.data()[1];
    elipse_z = manip_measure_in_data.data()[2];

    manip_stiffness_x =  manip_calculate(elipse_x);
    manip_stiffness_y =  manip_calculate(elipse_y);
    manip_stiffness_z =  manip_calculate(elipse_z);


    cart_stiffdamp_out_data.stiffness(0) = manip_stiffness_x;
    cart_stiffdamp_out_data.stiffness(3) = manip_stiffness_x;

    cart_stiffdamp_out_data.stiffness(1) = manip_stiffness_y;
    cart_stiffdamp_out_data.stiffness(4) = manip_stiffness_y;

    cart_stiffdamp_out_data.stiffness(2) = manip_stiffness_z;
    cart_stiffdamp_out_data.stiffness(5) = manip_stiffness_z;


    // Filter these output later
    cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);


}


double ManipToStiffnessXYZ::manip_calculate(double manip_factor){


    if(manip_factor < manip_min){
        manip_stiffness = stiffness_max;
    }

    else if(manip_factor > manip_max){
        manip_stiffness = stiffness_min;
    }

    else if(manip_factor <= manip_max && manip_factor >= manip_min)
    {

        manip_stiffness= ((stiffness_max-stiffness_min)/(manip_min-manip_max))*manip_factor+stiffness_max;

    }

    return manip_stiffness;



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
