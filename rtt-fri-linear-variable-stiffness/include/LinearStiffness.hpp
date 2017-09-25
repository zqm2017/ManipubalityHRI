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

class LinearStiffness : public RTT::TaskContext{
public:
    LinearStiffness(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool setCartPose(KDL::Vector position, KDL::Rotation orientation, double duration);
    void setRedRes(rstrt::kinematics::JointAngles joint_vals);

    void setCartStiffDamp(Eigen::VectorXf  cart_stiff, Eigen::VectorXf cart_damp);
    void stiffnessEstimator(double resultant_force);

private:

    /** OUTPUT PORTS **/
    RTT::OutputPort<rstrt::dynamics::JointImpedance> cart_stiffdamp_out_port;
    rstrt::dynamics::JointImpedance                  cart_stiffdamp_out_data;


    /** INPUT PORTS **/

    RTT::InputPort<rstrt::dynamics::Wrench>     cur_force_torq_in_port;
    RTT::FlowStatus                             cur_force_torq_in_flow;
    rstrt::dynamics::Wrench                     cur_force_torq_in_data;


    void initializePorts();


    // tasks:

    double resultant_force;
    double _stiffness;
   //std::vector _stiffness_vector_in, _stiffness_vector_out; // TODO-> replace this later with eigen vector
    double _filtered_stiffness,alpha,_a,_b,_xml,_stiffness_counter;
    double force_min, force_max, stiffness_min, stiffness_max;

   std::vector<double> _stiffness_vector_in,_stiffness_vector_out;
};
#endif
