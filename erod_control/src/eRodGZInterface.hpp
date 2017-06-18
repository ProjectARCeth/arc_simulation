#pragma once

#include <iostream>
#include"ackermann_msgs/AckermannDrive.h"
#include "arc_msgs/State.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

class GZInterface
{
public:
  // Constructor which sets up the needed Publishers and Subscribers.
  GZInterface(const ros::NodeHandle & nh);
  // Standard destructor.
  ~GZInterface();
// Callback functions.
  // saves the incoming state informations from gazebo to state in the needed format.
  void getGZState(const nav_msgs::Odometry::ConstPtr& gz_state);
  // saves the incoming steering and velocity command to refSteerAngle converts them with ackermann and publishes them to gazebo.
  void getCommand(const ackermann_msgs::AckermannDrive::ConstPtr& save_com);

private:
// Nodehandle.
  ros::NodeHandle nh_;

// Publishers.
  // Publisher for the steering angle Position for the front left wheel.
  ros::Publisher steer_angle_left_pub;
  // Publisher for the steering angle Position for the front right wheel.
  ros::Publisher steer_angle_right_pub;
  // Publishes Torquecommand to left rear wheel
  ros::Publisher torque_rear_left_pub;
  // Publishes Torquecommand to right rear wheel
  ros::Publisher torque_rear_right_pub;
  // Publishes the state from GZ to a topic which can be directly used by the core node of CAS (e.g PurePursuit).
  ros::Publisher to_cas_core_pub;
  // Publishes the generated discretized path of type nav_msgs/path to the /path topic and CAS can subscribe to it.
  ros::Publisher path_publisher;
//Subscriber
  // Subscribes to the output topic from Gazebo which publishes messages of type nav_msgs/Odometry.
  ros::Subscriber gz_state_sub;
  // Subscribes to the commands generated by the core node.
  ros::Subscriber from_cas_core_sub;

// Member variables
  //Variable where the desired velocity gets stored.
  float ref_velocity;
  // Variable where the desired steering angle for the bicycle model gets stored.
  float ref_steer_angle;
  // Variable which stores the desired steering angle for the front left wheel.
  std_msgs::Float64 left_steer_angle;
  // Variable which stores the desired steering angle for the front right wheel.
  std_msgs::Float64 right_steer_angle;
  // Variable which stores the torque command for the rear left wheel.
  std_msgs::Float64 torque_rear_left;
  // Variable which stores the torque command for the rear right wheel.
  std_msgs::Float64 torque_rear_right;
  // Variable which is used for the conversion of the state which comes from Gazebo and goes to the core node.
  arc_msgs::State conv_state;
  // Variable which stores the discretized path.
  nav_msgs::Path disc_path;
// Member methods
  void calcTorque();
};

/*
Notizen
Vierter Teilschritt: Dieses Objekt subscribed zu den nötigen Topics
um die Sollgeschwindigkeit zu holen

Fünfter Teilschritt: Dieses Objekt subscribed zum /gazebo/linkstates Topic,
konvertiert sie ins richtige Format und publiziert sie auf den richtigen Topics.

Sechster Teilschritt: Umrechnung zu Ackermann und publish über ackermann topic
*/
