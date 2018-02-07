#include "ConstStiffness.hpp"
#include <rtt/Component.hpp>
#include <iostream>

ConstStiffness::ConstStiffness(std::string const& name) : TaskContext(name){
    initializePorts();

    stiffness_x = 10;
    stiffness_y = 10;
    stiffness_z = 10;

    stiffness_A = 300;
    stiffness_B = 300;
    stiffness_C = 300;



    addProperty("stiffness_x",stiffness_x).doc("Stiffness value at Trans axis X");
    addProperty("stiffness_y",stiffness_y).doc("Stiffness value at Trans axis Y");
    addProperty("stiffness_z",stiffness_z).doc("Stiffness value at Trans axis Z");

    addProperty("stiffness_A",stiffness_A).doc("Stiffness value at ROT axis A");
    addProperty("stiffness_A",stiffness_B).doc("Stiffness value at ROT axis B");
    addProperty("stiffness_A",stiffness_C).doc("Stiffness value at ROT axis C");


}



bool ConstStiffness::configureHook(){
    return true;
}

bool ConstStiffness::startHook(){
    return true;
}


// Apply differential stiffness limits are XYZ and ABC : TODO
void ConstStiffness::updateHook(){




    cart_stiffdamp_out_data.stiffness(0) = stiffness_x;
    cart_stiffdamp_out_data.stiffness(1) = stiffness_y;
    cart_stiffdamp_out_data.stiffness(2) = stiffness_z;

    cart_stiffdamp_out_data.stiffness(3) = stiffness_A;
    cart_stiffdamp_out_data.stiffness(4) = stiffness_B;
    cart_stiffdamp_out_data.stiffness(5) = stiffness_C;

    cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);


}




void ConstStiffness::stopHook() {
}

void ConstStiffness::cleanupHook() {
}

void ConstStiffness::initializePorts(){

    /** OUTPUT PORTS    **/

    cart_stiffdamp_out_data = rstrt::dynamics::JointImpedance(6);
    cart_stiffdamp_out_port.setName("cart_stiffdamp_out_port");
    cart_stiffdamp_out_port.setDataSample(cart_stiffdamp_out_data);
    ports()->addPort(cart_stiffdamp_out_port);




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
ORO_CREATE_COMPONENT(ConstStiffness)
