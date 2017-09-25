#ifndef CARTESIANPOSEMANAGER_HPP
#define CARTESIANPOSEMANAGER_HPP

#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>
#include <QuinticPolynomial.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <math.h>
#define DOF_SIZE 7

class CartesianPoseManager : public RTT::TaskContext{
public:
    CartesianPoseManager(std::string const& name);
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

    RTT::OutputPort<rstrt::kinematics::JointAngles> red_res_out_port;
    rstrt::kinematics::JointAngles                  red_res_out_data;

    RTT::OutputPort<rstrt::dynamics::JointTorques> ext_trq_out_port;
    rstrt::dynamics::JointTorques                  ext_trq_out_data;

    RTT::OutputPort<KDL::Frame> cart_pose_out_port, cart_pose_loop_out_port;
    KDL::Frame                  cart_pose_out_data, cart_pose_loop_out_data;

    /** INPUT PORTS **/

    RTT::InputPort<KDL::Frame> cur_cart_pose_in_port;
    RTT::FlowStatus            cur_cart_pose_in_flow;
    KDL::Frame                 cur_cart_pose_in_data;

    RTT::InputPort<rstrt::dynamics::Wrench>     cur_force_torq_in_port;
    RTT::FlowStatus                             cur_force_torq_in_flow;
    rstrt::dynamics::Wrench                     cur_force_torq_in_data;

    RTT::InputPort<rstrt::dynamics::JointImpedance> des_cart_stiffdamp_in_port;
    RTT::FlowStatus                                 des_cart_stiffdamp_in_flow;
    rstrt::dynamics::JointImpedance                 des_cart_stiffdamp_in_data;



    void initializePorts();


    // tasks:
    QuinticPolynomial<float> task;
    Eigen::Vector3f init_pos, fin_pos, intermed_pos;
    Eigen::Quaternionf init_rot, fin_rot, intermed_rot;
    double start_time, end_time;
    double time;
    bool task_set, stiffdamp_set;
    double resultant_force;
    double _stiffness;
   //std::vector _stiffness_vector_in, _stiffness_vector_out; // TODO-> replace this later with eigen vector
    double _filtered_stiffness,alpha,_a,_b,_xml,_stiffness_counter;
    double force_min, force_max, stiffness_min, stiffness_max;

   std::vector<double> _stiffness_vector_in,_stiffness_vector_out;
};
#endif
