#include "../include/calculate_kinematics.h"

using namespace std;


CalculateKinematics::CalculateKinematics()
{
	angular_velocities = new double[2];
}

CalculateKinematics::~CalculateKinematics()
{
	delete angular_velocities;
}

void CalculateKinematics::set_params(double wheel_radius, double dist_between_wheels)
{
	pioneer_p3dx.WHEEL_RADIUS = wheel_radius;
	pioneer_p3dx.DISTANCE_BETWEEN_WHEELS = dist_between_wheels;
}

void CalculateKinematics::calculate_angular_velocities(double velocity, double steer_angle)
{
	double right_wheel_vel = velocity + pioneer_p3dx.DISTANCE_BETWEEN_WHEELS * steer_angle;
	double left_wheel_vel = velocity - pioneer_p3dx.DISTANCE_BETWEEN_WHEELS * steer_angle;
	double right_wheel_an_vel = right_wheel_vel / pioneer_p3dx.WHEEL_RADIUS;
	double left_wheel_an_vel = left_wheel_vel / pioneer_p3dx.WHEEL_RADIUS;

	angular_velocities[0] = right_wheel_an_vel;
	angular_velocities[1] = left_wheel_an_vel;
}

double* CalculateKinematics::get_angular_velocities()
{
	return angular_velocities;
}
