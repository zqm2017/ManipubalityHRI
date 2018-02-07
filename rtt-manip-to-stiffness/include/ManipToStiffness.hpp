#ifndef LINEARSTIFFNESS_HPP
#define LINEARSTIFFNESS_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <math.h>
#define DOF_SIZE 7

class ManipToStiffness : public RTT::TaskContext{
public:
    ManipToStiffness(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();


private:

    /** OUTPUT PORTS **/
    RTT::OutputPort<rstrt::dynamics::JointImpedance> cart_stiffdamp_out_port;
    rstrt::dynamics::JointImpedance                  cart_stiffdamp_out_data;


    /** INPUT PORTS **/

    RTT::InputPort<double>                      manip_measure_in_port;
    RTT::FlowStatus                             manip_measure_in_flow;
    double                                      manip_measure_in_data;



    void initializePorts();

    double manip_factor, manip_min, manip_max, stiffness_min, stiffness_max, max_rot_stiff;



    double _filtered_stiffness,alpha,_a,_b,_xml,_stiffness_counter;
    std::vector<double> _stiffness_vector_in,_stiffness_vector_out;
};
#endif
