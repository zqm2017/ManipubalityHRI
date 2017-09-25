#include "LinearStiffness.hpp"
#include <rtt/Component.hpp>
#include <iostream>

LinearStiffness::LinearStiffness(std::string const& name) : TaskContext(name){
    initializePorts();


    _stiffness_counter= 0;
    alpha=0.010;
    force_max = 15;
    force_min = 2;
    stiffness_max = 2200;
    stiffness_min = 10;


    addProperty("alpha",alpha).doc("The tuning parameter for the low pass filter");
    addProperty("force_max",force_max).doc("Max force for the linear heuristic");
    addProperty("force_min",force_min).doc("Min force for the linear heuristic");
    addProperty("stiffness_max",stiffness_max).doc("Max Stiffness for the linear heuristic");
    addProperty("stiffness_min",stiffness_min).doc("Min Stiffness for the linear heuristic");


}



bool LinearStiffness::configureHook(){
    return true;
}

bool LinearStiffness::startHook(){
    return true;
}

void LinearStiffness::updateHook(){

    cur_force_torq_in_flow = cur_force_torq_in_port.read(cur_force_torq_in_data);
    resultant_force = sqrt(pow(cur_force_torq_in_data.forces.data()[0],2)+pow(cur_force_torq_in_data.forces.data()[1],2)+pow(cur_force_torq_in_data.forces.data()[2],2));

    //std::cout<<"Resultant: "<<resultant_force<<std::endl;

    stiffnessEstimator(resultant_force);

    for(int i = 0 ; i < 6 ; ++i){
        cart_stiffdamp_out_data.stiffness(i) = _filtered_stiffness;

    }
    //std::cout<<"Stiffness: "<<_filtered_stiffness<<std::endl;

    cart_stiffdamp_out_port.write(cart_stiffdamp_out_data);

}


void LinearStiffness::stiffnessEstimator(double resultant_force)
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


void LinearStiffness::stopHook() {
}

void LinearStiffness::cleanupHook() {
}

void LinearStiffness::initializePorts(){

    /** OUTPUT PORTS    **/

    cart_stiffdamp_out_data = rstrt::dynamics::JointImpedance(6);
    cart_stiffdamp_out_port.setName("cart_stiffdamp_out_port");
    cart_stiffdamp_out_port.setDataSample(cart_stiffdamp_out_data);
    ports()->addPort(cart_stiffdamp_out_port);

    /** INPUT PORTS **/

    cur_force_torq_in_flow = RTT::NoData;
    cur_force_torq_in_port.setName("cur_force_torque_in");
    cur_force_torq_in_data = rstrt::dynamics::Wrench();
    ports()->addPort(cur_force_torq_in_port);



}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(LinearStiffness)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(LinearStiffness)
