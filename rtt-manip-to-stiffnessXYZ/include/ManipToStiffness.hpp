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

    double manip_stiffness, manip_stiffness_x, manip_stiffness_y, manip_stiffness_z, scale_min, scale_max,scale_factor, stiffness_min, stiffness_max_trans, stiffness_max_rot;

    double elipse_x, elipse_y, elipse_z, max_elipse, scaled_x, scaled_y, scaled_z;

    double manip_calculate(double);


};
#endif
