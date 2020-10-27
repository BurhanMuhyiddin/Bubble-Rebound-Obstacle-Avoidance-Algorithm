#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "../include/calculate_kinematics.h"

using namespace std;

#define PI	3.14159265359

class AvoidObstacle
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	CalculateKinematics ck;
	int N;
	double velocity;
	double bubble_boundary, rebound_angle;
	double* alphas;

public:
	AvoidObstacle();
	void readSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void calculate_rebound_angle(const std_msgs::Float32MultiArray::ConstPtr& msg);
	~AvoidObstacle();
};

AvoidObstacle::AvoidObstacle()
{
	ck.set_params(0.0975, 0.3810); // set robot parameters for kinematics calculations
	N = 8; // set number of sensors
	
	alphas = new double[N/2];
	alphas[0] = PI*90.0/180.0;
	alphas[1] = PI*50.0/180.0;
    alphas[2] =	PI*30.0/180.0;
    alphas[3] =	PI*10.0/180.0;

	velocity = 0.06; // set linear velocity of the robot
	rebound_angle = 0.0; // initialize rebound_angle with 0
	bubble_boundary = 0.6; // define max distance to the obstacle

	sub = nh.subscribe("/ultrasonic_sensors", 1000, &AvoidObstacle::readSensorCallback, this);
	pub = nh.advertise<std_msgs::Float32MultiArray>("/angular_velocities", 10);
}

AvoidObstacle::~AvoidObstacle()
{
	delete alphas;
}

void AvoidObstacle::calculate_rebound_angle(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	double sum_angle = 0.0;
	double sum_dist = 0.0;
	
	for (int i = 0; i < N/2; i++)
	{
		sum_angle += alphas[i] * (msg->data[i] - msg->data[N-1-i]);
	}
	for (int i = 0; i < N; i++)
	{
		sum_dist += msg->data[i];
	}

	rebound_angle = (double) (sum_angle / sum_dist);
}

void AvoidObstacle::readSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	// Check whether an object exists in range of boundary
	for (int i = 0; i < N; i++)
	{
		if (msg->data[i] > bubble_boundary)
		{
			rebound_angle = 0.0; // if no obstacle continue without changing direction
			continue;
		}
		else
		{
			calculate_rebound_angle(msg); // if obstacle, calculate rebound angle
			break;
		}
	}

	ck.calculate_angular_velocities(velocity, rebound_angle);
	double* angular_velocities = ck.get_angular_velocities();
	std_msgs::Float32MultiArray new_msg;
	new_msg.data.push_back(angular_velocities[0]);
	new_msg.data.push_back(angular_velocities[1]);
	pub.publish(new_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_avoider");
	AvoidObstacle ao;

	ros::spin();
	
	return 0;
}
