#ifndef MANIPESTIMATOR_HPP
#define MANIPESTIMATOR_HPP
#include <rtt/RTT.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/frames.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <kdl/chain.hpp>
#include <math.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <geometry_msgs/Pose.h>
#include <rst-rt/geometry/Pose.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

class ManipEstimator : public RTT::TaskContext{
public:
    ManipEstimator(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();


private:

    /** OUTPUT PORTS **/

    // Manipulability measure port

    RTT::OutputPort<Eigen::Vector3d>    manip_elipse_out_port;
    Eigen::Vector3d                     manip_elipse_out_data;

    RTT::OutputPort<double>             manip_measure_out_port;
    double                              manip_measure_out_data;


    /** INPUT PORTS **/



    // Shoulder position with respect to camera frame


    RTT::InputPort<geometry_msgs::Pose> cur_shoulder_pose_in_port;
    RTT::FlowStatus                       cur_shoulder_pose_in_flow;
    geometry_msgs::Pose                cur_shoulder_pose_in_data;

    RTT::InputPort<geometry_msgs::Pose> cur_grip_pose_in_port;
    RTT::FlowStatus                       cur_grip_pose_in_flow;
    geometry_msgs::Pose                 cur_grip_pose_in_data;



    void initializePorts();

    KDL::Tree hand_tree, robot_tree;
    urdf::Model hand_model, robot_model;
    //std::string hand_urdf_path, robot_urdf_path;

    std::string hand_base,hand_eef,hand_urdf_path,robot_base,robot_eef,robot_urdf_path;
    KDL::Chain hand_chain, robot_chain;

    KDL::ChainFkSolverPos_recursive* fk_solver;

    //KDL::ChainIkSolverPos_LMA ik_solver; // decide on which IK solver to use, going with ChainIKSolverPosNR for time being

    KDL::Frame cart_pose_out, cur_target_cart_pose;
    KDL::JntArray q, q_hand_init, q_hand_out;

    KDL::ChainJntToJacSolver* jnt_to_jac_solver;

    KDL::Jacobian hand_jac;

    KDL::Tree hand_tree_updated;
    KDL::Chain hand_chain_updated;

    double manipulability ;
    Eigen::Vector3d  me_axis;
    void createKinChain(KDL::Frame);

    KDL::Vector shoulder_pos, grip_pos;

    KDL::Rotation shoulder_rot, grip_rot;

    Eigen::MatrixXd weights;

    // Output port for the visualization of joint states into ROS
    RTT::OutputPort<sensor_msgs::JointState> arm_conf_out_port;
    sensor_msgs::JointState arm_conf_out_data;

    // Output port for manipulability visualization
    RTT::OutputPort<double> manip_ros_out_port;
    double manip_ros_out_data;


    // Filter_param

    double _a,_b,_xml, _filter_counter, _filter_input, alpha;
    std::vector<double> _filter_vector_in,_filter_vector_out;
    // Force transmission
    Eigen::Vector3d u,_u;

};
#endif
