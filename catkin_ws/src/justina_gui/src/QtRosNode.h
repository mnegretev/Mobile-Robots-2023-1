#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf/transform_listener.h"
#include "custom_msgs/InverseKinematicsPose2Traj.h"
#include "custom_msgs/InverseKinematicsPose2Pose.h"
#include "custom_msgs/InverseKinematics.h"
#include "custom_msgs/ForwardKinematics.h"
#include "custom_msgs/GetPolynomialTrajectory.h"
#include "custom_msgs/FindLines.h"
#include "custom_msgs/FindObject.h"
#include "custom_msgs/TrainObject.h"
#include "custom_msgs/RecognizeObjects.h"
#include "custom_msgs/RecognizeObject.h"
#include "custom_msgs/SmoothPath.h"
#include "custom_msgs/RecognizedSpeech.h"
#include "sound_play/SoundRequest.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubCmdVel;
    ros::Publisher pubTorso;
    ros::Publisher pubLaGoalQ;
    ros::Publisher pubRaGoalQ;
    ros::Publisher pubLaGoalTraj;
    ros::Publisher pubRaGoalTraj;
    ros::Publisher pubHdGoalQ;
    ros::Publisher pubLaGoalGrip;
    ros::Publisher pubRaGoalGrip;
    ros::Publisher pubSay;
    ros::Subscriber subLaCurrentQ;
    ros::Subscriber subLaVoltage;
    ros::Subscriber subRaCurrentQ;
    ros::Subscriber subRaVoltage;
    ros::Subscriber subRecogSpeech;
    ros::ServiceClient cltAStar;
    ros::ServiceClient cltSmoothPath;
    ros::ServiceClient cltLaIKPose2Pose;
    ros::ServiceClient cltRaIKPose2Pose;
    ros::ServiceClient cltLaIKPose2Traj;
    ros::ServiceClient cltRaIKPose2Traj;
    ros::ServiceClient cltLaForwardKinematics;
    ros::ServiceClient cltRaForwardKinematics;
    ros::ServiceClient cltLaInverseKinematics;
    ros::ServiceClient cltRaInverseKinematics;
    ros::ServiceClient cltGetPolynomialTraj;
    ros::ServiceClient cltFindLines;
    ros::ServiceClient cltFindObject;
    ros::ServiceClient cltTrainObject;
    ros::ServiceClient cltRecogObjects;
    ros::ServiceClient cltRecogObject;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    std::vector<double> la_current_q;
    std::vector<double> ra_current_q;
    std::vector<double> la_current_cartesian;
    std::vector<double> ra_current_cartesian;
    double la_voltage;
    double ra_voltage;
    std::string recognized_sentence;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);
    void set_param_inflation_radius(float inflation_radius);
    void set_param_cost_radius(float cost_radius);
    void set_param_smoothing_alpha(float smoothing_alpha);
    void set_param_smoothing_beta(float  smoothing_beta);
    void call_a_start_path(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::Path& path);
    void call_smooth_path(nav_msgs::Path& path, nav_msgs::Path& smoothed_path);

    void publish_torso_position(float tr);
    void publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_la_goal_trajectory(trajectory_msgs::JointTrajectory Q);
    void publish_ra_goal_trajectory(trajectory_msgs::JointTrajectory Q);
    void publish_la_grip_angles(float a);
    void publish_ra_grip_angles(float a);
    void publish_head_angles(double pan, double tilt);
    void publish_say(std::string text_to_say);
    void callback_la_current_q(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void callback_la_voltage(const std_msgs::Float64::ConstPtr& msg);
    void callback_ra_current_q(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void callback_ra_voltage(const std_msgs::Float64::ConstPtr& msg);
    void callback_recog_speech(const custom_msgs::RecognizedSpeech::ConstPtr& msg);
    bool call_la_ik_trajectory(std::vector<double>& cartesian, trajectory_msgs::JointTrajectory& trajectory);
    bool call_ra_ik_trajectory(std::vector<double>& cartesian, trajectory_msgs::JointTrajectory& trajectory);
    bool call_la_ik_pose(std::vector<double>& cartesian, std::vector<double>& articular);
    bool call_ra_ik_pose(std::vector<double>& cartesian, std::vector<double>& articular);
    bool call_la_forward_kinematics(std::vector<double>& articular, std::vector<double>& cartesian);
    bool call_ra_forward_kinematics(std::vector<double>& articular, std::vector<double>& cartesian);
    bool call_la_inverse_kinematics(std::vector<double>& cartesian, std::vector<double>& articular);
    bool call_ra_inverse_kinematics(std::vector<double>& cartesian, std::vector<double>& articular);
    bool call_get_polynomial_traj(std::vector<double>& p1, std::vector<double>& p2, trajectory_msgs::JointTrajectory& trajectory);

    bool call_find_lines();
    bool call_find_object(std::string name);
    bool call_train_object(std::string name);
    bool call_recognize_objects();
    bool call_recognize_object(std::string name);
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
