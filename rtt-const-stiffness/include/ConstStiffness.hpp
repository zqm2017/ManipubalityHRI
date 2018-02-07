#ifndef CONSTSTIFFNESS_HPP
#define CONSTSTIFFNESS_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <math.h>
#define DOF_SIZE 7

class ConstStiffness : public RTT::TaskContext{
public:
    ConstStiffness(std::string const& name);
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

   // RTT::InputPort<double>                      manip_measure_in_port;
    //RTT::FlowStatus                             manip_measure_in_flow;
    //double                                      manip_measure_in_data;



    void initializePorts();

    double stiffness_x, stiffness_y, stiffness_z, stiffness_A, stiffness_B,stiffness_C;


};
#endif
