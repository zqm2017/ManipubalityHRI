#include "test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rostopic.h>

test_bridge::test_bridge(std::string const& name) : TaskContext(name){

    initializePorts();

}


bool test_bridge::configureHook(){




    return true;
}

bool test_bridge::startHook(){
    inport.createStream(rtt_roscomm::topic("grip_pose"));

    inport2.createStream(rtt_roscomm::topic("shoulder_pose"));

    return true;
}

void test_bridge::updateHook(){
    inflow = inport.read(indata);
    //std::cout<<"Pose1   : "<<indata.position.x<<"\t"<<indata.position.y<<"\t"<<indata.position.z<<std::endl;
    //std::cout<<"Orient1 : "<<indata.orientation.x<<"\t"<<indata.orientation.y<<"\t"<<indata.orientation.z<<"\t"<<indata.orientation.w<<std::endl;

    inflow2 = inport2.read(indata2);
    //std::cout<<"Pose2   : "<<indata2.position.x<<"\t"<<indata2.position.y<<"\t"<<indata2.position.z<<std::endl;
    //std::cout<<"Orient2 : "<<indata2.orientation.x<<"\t"<<indata2.orientation.y<<"\t"<<indata2.orientation.z<<"\t"<<indata2.orientation.w<<std::endl;

    outport_grip.write(indata);
    outport_shoulder.write(indata2);

}



void test_bridge::stopHook() {

}

void test_bridge::cleanupHook() {

}


void test_bridge::initializePorts(){


   /* manip_measure_out_data = double();
    manip_measure_out_port.setName("manip_measure_out_port");
    manip_measure_out_port.setDataSample(manip_measure_out_data);
    ports()->addPort(manip_measure_out_port);
*/



    inflow = RTT::NoData;
    inport.setName(" Grip Pose Port");
    indata = geometry_msgs::Pose();
    ports()->addPort(inport);

    inflow2 = RTT::NoData;
    inport2.setName(" Shoulder Pose Port");
    indata2 = geometry_msgs::Pose();
    ports()->addPort(inport2);


    outdata_grip  = geometry_msgs::Pose();
    outport_grip.setName("outport_grip");
    outport_grip.setDataSample(outdata_grip);
    ports()->addPort(outport_grip);

    outdata_shoulder  = geometry_msgs::Pose();
    outport_shoulder.setName("outport_shoulder");
    outport_shoulder.setDataSample(outdata_shoulder);
    ports()->addPort(outport_shoulder);



}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(test_bridge)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(test_bridge)
