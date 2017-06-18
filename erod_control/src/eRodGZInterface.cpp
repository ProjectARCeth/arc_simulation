#include "eRodGZInterface.hpp"

// Constructor.
GZInterface::GZInterface(const ros::NodeHandle &nh) : nh_(nh)
{
  ROS_INFO("Initializing the Gazebo Interface");

  //Publishers
  steer_angle_left_pub = nh_.advertise<std_msgs::Float64>("/erod/joint1_position_controller/command", 10);
  steer_angle_right_pub = nh_.advertise<std_msgs::Float64>("/erod/joint2_position_controller/command", 10);
  torque_rear_left_pub = nh_.advertise<std_msgs::Float64>("/erod/leftRearWheel_effort_controller/command", 10);
  torque_rear_right_pub = nh_.advertise<std_msgs::Float64>("/erod/rightRearWheel_effort_controller/command", 10);
  // The converted state in the correct format (arc_msgs/State) gets published on the same topic as L&M will publish the real state.
  to_cas_core_pub = nh_.advertise<arc_msgs::State>("/state", 10);
  // Publishes the path to CAS core.
  path_publisher=nh_.advertise<nav_msgs::Path>("/path",10);
  // Subscriber.
  // Subscribes to the topic from Gazebo.
  gz_state_sub = nh_.subscribe("/odom", 100, &GZInterface::getGZState, this);
  from_cas_core_sub = nh_.subscribe("/stellgroessen_safe",100, &GZInterface::getCommand, this);

  ROS_INFO("Gazebo Interface initialized!");
}

//Standard destructor
GZInterface::~GZInterface() {}

//member methods

// Callback function which processes the incoming data from Gazebo by storing them into a member variable of type arc_msgs/State and reroutes them to the core node.
void GZInterface::getGZState(const nav_msgs::Odometry::ConstPtr& gz_state)
{
  //put gzState of msg-type nav_msgs::Odometry into into state of msg-type arc_msgs::State and publish them to CAS core
  conv_state.pose.pose.position.x = gz_state->pose.pose.position.x;
  conv_state.pose.pose.position.y = gz_state->pose.pose.position.y;
  conv_state.pose.pose.position.z = gz_state->pose.pose.position.z;

  conv_state.pose.pose.orientation.x = gz_state->pose.pose.orientation.x;
  conv_state.pose.pose.orientation.y = gz_state->pose.pose.orientation.y;
  conv_state.pose.pose.orientation.z = gz_state->pose.pose.orientation.z;
  conv_state.pose.pose.orientation.w = gz_state->pose.pose.orientation.w;

  conv_state.pose_diff.twist.linear.x = gz_state->twist.twist.linear.x;
  conv_state.pose_diff.twist.linear.y = gz_state->twist.twist.linear.y;
  conv_state.pose_diff.twist.linear.z = gz_state->twist.twist.linear.z;

  conv_state.pose_diff.twist.angular.x = gz_state->twist.twist.angular.x;
  conv_state.pose_diff.twist.angular.y = gz_state->twist.twist.angular.y;
  conv_state.pose_diff.twist.angular.z = gz_state->twist.twist.angular.z;

  conv_state.current_arrayposition = NAN;
  conv_state.stop = true;

  to_cas_core_pub.publish(conv_state);

}
// Callback function which gets the commands from the core node and reroutes them to Gazebo
void GZInterface::getCommand(const ackermann_msgs::AckermannDrive::ConstPtr& save_com)
{
  ref_steer_angle = save_com->steering_angle;
  ref_velocity =  1;           //save_com->speed;

  left_steer_angle.data = ref_steer_angle;
  right_steer_angle.data = ref_steer_angle;

  steer_angle_left_pub.publish(left_steer_angle);
  steer_angle_right_pub.publish(right_steer_angle);

  float torque;
  torque = 0.5;//                          k_p*(ref_velocity-v_state);
  torque_rear_left.data = torque;
  torque_rear_right.data = -torque;
  torque_rear_left_pub.publish(torque_rear_left);
  torque_rear_right_pub.publish(torque_rear_right);
  //calcTorque();
}

// PID controller for ref_velocity-->wheel_torque
void GZInterface::calcTorque()
{
  float k_p = 1;
  float v_x = conv_state.pose_diff.twist.linear.x;
  float v_y = conv_state.pose_diff.twist.linear.y;
  float v_z = conv_state.pose_diff.twist.linear.z;
  float v_state  = sqrt(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));
  //ROS_INFO("Ready to add two ints.");
  float torque;
  std::cout<<v_state<<std::endl;
  torque = 1.0;//                          k_p*(ref_velocity-v_state);
  torque_rear_left.data = torque;
  torque_rear_left.data = 0.0;  //-torque;
  torque_rear_left_pub.publish(torque_rear_left);
  torque_rear_right_pub.publish(torque_rear_right);
}
