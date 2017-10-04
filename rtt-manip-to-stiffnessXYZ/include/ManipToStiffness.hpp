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

class ManipToStiffnessXYZ : public RTT::TaskContext{
public:
    ManipToStiffnessXYZ(std::string const& name);
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

    RTT::InputPort<Eigen::Vector3d>             manip_measure_in_port;
    RTT::FlowStatus                             manip_measure_in_flow;
    Eigen::Vector3d                             manip_measure_in_data;



    void initializePorts();

    double manip_stiffness, manip_stiffness_x, manip_stiffness_y, manip_stiffness_z, manip_min, manip_max, stiffness_min, stiffness_max;

    double elipse_x, elipse_y, elipse_z;

    double manip_calculate(double);


};
#endif
