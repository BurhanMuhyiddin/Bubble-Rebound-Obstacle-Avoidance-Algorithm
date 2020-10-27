#pragma once

class CalculateKinematics
{
private:
	struct parameters
	{
		double WHEEL_RADIUS;
		double DISTANCE_BETWEEN_WHEELS;
	};
	parameters pioneer_p3dx;

	double* angular_velocities;

public:
	CalculateKinematics();
	~CalculateKinematics();
	void set_params(double wheel_radius, double dist_between_wheels);
	void calculate_angular_velocities(double velocity, double steer_angle);
	double* get_angular_velocities();
};
