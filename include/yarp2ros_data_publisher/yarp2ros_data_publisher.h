#ifndef YARP2ROS_DATA_PUBLISHER_H
#define YARP2ROS_DATA_PUBLISHER_H


#include <iostream>
#include <iomanip>

#include "ros/ros.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace Eigen;
using namespace yarp::os;

typedef Matrix<double, 6, 1> Vector6d;


class yarp2ros_data_publisher
{

	protected: 

		// ROS VARIABLES:
		// A handle to the node in ros
		ros::NodeHandle nh_;
		// Rate of the run loop
		ros::Rate loop_rate_;

		// moduleName
		std::string robot_name;


		// Yarp ports to read the data from:

		// Force/torque sensors ports arms and feet  
	    BufferedPort<Bottle> left_arm_FT_inputPort;
	    BufferedPort<Bottle> right_arm_FT_inputPort;

	    BufferedPort<Bottle> left_foot_FT_inputPort;
	    BufferedPort<Bottle> right_foot_FT_inputPort;

	    Bottle *left_arm_FT_data;
	    Bottle *right_arm_FT_data;

	    Bottle *left_foot_FT_data;
	    Bottle *right_foot_FT_data;

	    // feet F/T measurements
	    Vector6d left_arm_FT_vector;
	    Vector6d right_arm_FT_vector;
	    //
	   	Vector6d left_foot_FT_vector;
	    Vector6d right_foot_FT_vector;


		// Publishers:

		// Publisher for the left arm force/torque measurements in [N]
		ros::Publisher pub_left_arm_force_torque_;
		// Publisher for the right arm force/torque measurements in [N]
		ros::Publisher pub_right_arm_force_torque_;
		// Publisher for the left foot force/torque measurements in [N]
		ros::Publisher pub_left_foot_force_torque_;
		// Publisher for the right foot force/torque measurements in [N]
		ros::Publisher pub_right_foot_force_torque_;


		

		//
	

	public :
		//
		//
		yarp2ros_data_publisher(ros::NodeHandle &n, double frequency, 
								std::string robot_name_);

		~yarp2ros_data_publisher();

		// Publish methods
		void publish_arms_force_torque_measurements();

		void publish_feet_force_torque_measurements();


		// getter
		bool get_arms_force_torque_values();
		bool get_feet_force_torque_values();

		void closePorts();

};

#endif // YARP2ROS_DATA_PUBLISHER_H




