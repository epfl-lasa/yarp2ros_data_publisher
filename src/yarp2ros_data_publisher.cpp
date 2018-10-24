
#include "yarp2ros_data_publisher/yarp2ros_data_publisher.h"

using namespace std;
using namespace Eigen;
using namespace yarp::os;


// ------------------
yarp2ros_data_publisher::yarp2ros_data_publisher(ros::NodeHandle &n, double frequency, std::string robot_name_)	: nh_(n) 
																												, loop_rate_(frequency)
																												, robot_name(robot_name_)
																										
{
	// names of topics to be published
	std::string topic_left_arm_force_torque_   = robot_name + "_left_arm_wrench";
	std::string topic_right_arm_force_torque_  = robot_name + "_right_arm_wrench";
	std::string topic_left_foot_force_torque_  = robot_name + "_left_foot_wrench";
	std::string topic_right_foot_force_torque_ = robot_name + "_right_foot_wrench";

	// Publishers:
	pub_left_arm_force_torque_ 		= nh_.advertise<geometry_msgs::WrenchStamped>(topic_left_arm_force_torque_, 5);
	pub_right_arm_force_torque_  	= nh_.advertise<geometry_msgs::WrenchStamped>(topic_right_arm_force_torque_, 5);
	pub_left_foot_force_torque_  	= nh_.advertise<geometry_msgs::WrenchStamped>(topic_left_foot_force_torque_, 5);
	pub_right_foot_force_torque_  	= nh_.advertise<geometry_msgs::WrenchStamped>(topic_right_foot_force_torque_, 5);


	// Opening force/torque sensors ports and establish connection
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // local port
    //-----------
    left_arm_FT_inputPort.open("/left_arm_FT_readPort");
    right_arm_FT_inputPort.open("/right_arm_FT_readPort");
    left_foot_FT_inputPort.open("/left_foot_FT_readPort");
    right_foot_FT_inputPort.open("/right_foot_FT_readPort");

    // remote port (on the robot side)
    //--------------------------------
    std::string l_arm_FT_portName="/";
	            l_arm_FT_portName += robot_name;
	            l_arm_FT_portName += "/left_arm/analog:o";

    std::string r_arm_FT_portName="/";
	            r_arm_FT_portName += robot_name;
	            r_arm_FT_portName += "/right_arm/analog:o";
    
    std::string l_foot_FT_portName ="/";
	            l_foot_FT_portName += robot_name;
	            l_foot_FT_portName += "/left_foot/analog:o";
    
    std::string r_foot_FT_portName="/";
	            r_foot_FT_portName += robot_name;
	            r_foot_FT_portName += "/right_foot/analog:o";

	// connection to the ports
	// ------------------------
	Network::connect(l_arm_FT_portName.c_str(),	 left_arm_FT_inputPort.getName().c_str());
    Network::connect(r_arm_FT_portName.c_str(),	 right_arm_FT_inputPort.getName().c_str());
    Network::connect(l_foot_FT_portName.c_str(), left_foot_FT_inputPort.getName().c_str());
    Network::connect(r_foot_FT_portName.c_str(), right_foot_FT_inputPort.getName().c_str());
   
	// Reading of all ports and initilize vectors of data
	left_arm_FT_data  	= left_arm_FT_inputPort.read();
    right_arm_FT_data  	= right_arm_FT_inputPort.read();   
    left_foot_FT_data 	= left_foot_FT_inputPort.read();
    right_foot_FT_data 	= right_foot_FT_inputPort.read();   
    //
    // Resizing of the measurement vectors    
	left_arm_FT_vector.resize(left_arm_FT_data->size());		left_arm_FT_vector.setZero();
    right_arm_FT_vector.resize(right_arm_FT_data->size());		right_arm_FT_vector.setZero();

    left_foot_FT_vector.resize(left_foot_FT_data->size());		left_foot_FT_vector.setZero();
    right_foot_FT_vector.resize(right_foot_FT_data->size());	right_foot_FT_vector.setZero();


}

yarp2ros_data_publisher::~yarp2ros_data_publisher()
{
    //
    left_arm_FT_inputPort.close();
    right_arm_FT_inputPort.close();

    left_foot_FT_inputPort.close();
    right_foot_FT_inputPort.close();
	
}


bool yarp2ros_data_publisher::get_arms_force_torque_values()
{
	//
	left_arm_FT_data   = left_arm_FT_inputPort.read();
	right_arm_FT_data  = right_arm_FT_inputPort.read();

    for (int i=0; i<left_arm_FT_data->size(); i++) 
    {
        left_arm_FT_vector(i)  = left_arm_FT_data->get(i).asDouble();
        right_arm_FT_vector(i) = right_arm_FT_data->get(i).asDouble();
    }

	return true;
}

bool yarp2ros_data_publisher::get_feet_force_torque_values()
{
	//
	//
	left_foot_FT_data   = left_foot_FT_inputPort.read();
	right_foot_FT_data  = right_foot_FT_inputPort.read();

    for (int i=0; i<left_foot_FT_data->size(); i++) 
    {
        left_foot_FT_vector(i)  = left_foot_FT_data->get(i).asDouble();
        right_foot_FT_vector(i) = right_foot_FT_data->get(i).asDouble();
    }

	return true;
}



// publishing of the reference trajectories
void yarp2ros_data_publisher::publish_arms_force_torque_measurements()
{
		
	geometry_msgs::WrenchStamped msg_arm_wrench;
	//

	msg_arm_wrench.header.stamp    = ros::Time::now();
	msg_arm_wrench.header.frame_id = "left_arm_FTsensor_link";
	msg_arm_wrench.wrench.force.x  = left_arm_FT_vector(0);
	msg_arm_wrench.wrench.force.y  = left_arm_FT_vector(1);
	msg_arm_wrench.wrench.force.z  = left_arm_FT_vector(2);
	msg_arm_wrench.wrench.torque.x = left_arm_FT_vector(3);
	msg_arm_wrench.wrench.torque.y = left_arm_FT_vector(4);
	msg_arm_wrench.wrench.torque.z = left_arm_FT_vector(5);

	// publishing the messages
	pub_left_arm_force_torque_.publish(msg_arm_wrench);


	msg_arm_wrench.header.stamp    = ros::Time::now();
	msg_arm_wrench.header.frame_id = "right_arm_FTsensor_link";
	msg_arm_wrench.wrench.force.x  = right_arm_FT_vector(0);
	msg_arm_wrench.wrench.force.y  = right_arm_FT_vector(1);
	msg_arm_wrench.wrench.force.z  = right_arm_FT_vector(2);
	msg_arm_wrench.wrench.torque.x = right_arm_FT_vector(3);
	msg_arm_wrench.wrench.torque.y = right_arm_FT_vector(4);
	msg_arm_wrench.wrench.torque.z = right_arm_FT_vector(5);

	// publishing the messages
	pub_right_arm_force_torque_.publish(msg_arm_wrench);

}


// publishing of the reference trajectories
void yarp2ros_data_publisher::publish_feet_force_torque_measurements()
{
		
	
	geometry_msgs::WrenchStamped msg_foot_wrench;
	//
	
	msg_foot_wrench.header.stamp    = ros::Time::now();
	msg_foot_wrench.header.frame_id = "left_foot_FTsensor_link";
	msg_foot_wrench.wrench.force.x  = left_foot_FT_vector(0);
	msg_foot_wrench.wrench.force.y  = left_foot_FT_vector(1);
	msg_foot_wrench.wrench.force.z  = left_foot_FT_vector(2);
	msg_foot_wrench.wrench.torque.x = left_foot_FT_vector(3);
	msg_foot_wrench.wrench.torque.y = left_foot_FT_vector(4);
	msg_foot_wrench.wrench.torque.z = left_foot_FT_vector(5);

	// publishing the messages
	pub_left_foot_force_torque_.publish(msg_foot_wrench);


	msg_foot_wrench.header.stamp    = ros::Time::now();
	msg_foot_wrench.header.frame_id = "right_foot_FTsensor_link";
	msg_foot_wrench.wrench.force.x  = right_foot_FT_vector(0);
	msg_foot_wrench.wrench.force.y  = right_foot_FT_vector(1);
	msg_foot_wrench.wrench.force.z  = right_foot_FT_vector(2);
	msg_foot_wrench.wrench.torque.x = right_foot_FT_vector(3);
	msg_foot_wrench.wrench.torque.y = right_foot_FT_vector(4);
	msg_foot_wrench.wrench.torque.z = right_foot_FT_vector(5);

	// publishing the messages
	pub_right_foot_force_torque_.publish(msg_foot_wrench);


}

void yarp2ros_data_publisher::closePorts()
{
	//
    left_arm_FT_inputPort.close();
    right_arm_FT_inputPort.close();

    left_foot_FT_inputPort.close();
    right_foot_FT_inputPort.close();
}