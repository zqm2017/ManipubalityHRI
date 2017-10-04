#include "ManipToStiffness.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ManipToStiffness::ManipToStiffness(std::string const& name) : TaskContext(name){
    initializePorts();

    stiffness_max = 2200; // ToDo: come up with reasonable values
    stiffness_min = 10;
    manip_max = 4;
    manip_min = 0.01;



    addProperty("manip_factor",manip_factor).doc("The manipulability factor for chaning stiffness");
    addProperty("stiffness_max",stiffness_max).doc("Max Stiffness for the linear heuristic");
    addProperty("stiffness_min",stiffness_min).doc("Min Stiffness for the linear heuristic");
    addProperty("manip_max",manip_max).doc("Max manipulability expected");
    addProperty("manip_min",manip_min).doc("Min manipulability expected");


}



bool ManipToStiffness::configureHook(){
    return true;
}

bool ManipToStiffness::startHook(){
    return true;
}

void ManipToStiffness::updateHook(){

    manip_measure_in_flow = manip_measure_in_port.read(manip_measure_in_data);

    if(manip_measure_in_data < manip_min){
        manip_factor = stiffness_max;
    }

    else if(manip_measure_in_data > manip_max){
        manip_factor = stiffness_min;
    }

    else if(manip_measure_in_data <= manip_max && manip_measure_in_data >= manip_min)
    {

        manip_factor = ((stiffness_max-stiffness_min)/(manip_min-manip_max))*manip_measure_in_data+stiffness_max;

    }


    for(int i = 0 ; i < 6 ; ++i){
        cart_stiffdamp_out_data.stiffness(i) = manip_factor;

    }

    // Filter these output later
    cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);


}




void ManipToStiffness::stopHook() {
}

void ManipToStiffness::cleanupHook() {
}

void ManipToStiffness::initializePorts(){

    /** OUTPUT PORTS    **/

    cart_stiffdamp_out_data = rstrt::dynamics::JointImpedance(6);
    cart_stiffdamp_out_port.setName("cart_stiffdamp_out_port");
    cart_stiffdamp_out_port.setDataSample(cart_stiffdamp_out_data);
    ports()->addPort(cart_stiffdamp_out_port);

    /** INPUT PORTS **/

    manip_measure_in_flow = RTT::NoData;
    manip_measure_in_port.setName("manip_measure_in_port");
    manip_measure_in_data = double();
    ports()->addPort(manip_measure_in_port);



}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ManipToStiffness)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ManipToStiffness)