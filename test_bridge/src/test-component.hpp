#ifndef OROCOS_TEST_COMPONENT_HPP
#define OROCOS_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <geometry_msgs/Pose.h>


class test_bridge : public RTT::TaskContext{
public:
    test_bridge(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:

    void initializePorts();
    RTT::InputPort<geometry_msgs::Pose> inport;
    RTT::FlowStatus inflow;
    geometry_msgs::Pose indata;


    RTT::InputPort<geometry_msgs::Pose> inport2;
    RTT::FlowStatus inflow2;
    geometry_msgs::Pose indata2;


    RTT::OutputPort<geometry_msgs::Pose> outport_shoulder;
    RTT::FlowStatus outflow_shoulder;
    geometry_msgs::Pose outdata_shoulder;


    RTT::OutputPort<geometry_msgs::Pose> outport_grip;
    RTT::FlowStatus outflow_grip;
    geometry_msgs::Pose outdata_grip;
};
#endif
