#ifndef ROBO_CONTROL
#define ROBO_CONTROL
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include "mat_kin.h"
#include "ensc-488.h"
#include "traj_plan.h"

#define HEIGHT 4
#define WIDTH 4
#define PI 3.14159265359

class robo_control
{
private:
	mat_kin cmd;
	traj_plan cmd2;

	JOINT* q0;
	JOINT* q1;

	double j1, j2, j3, j4;
	double x, y, z, theta;
	double** internal_form;
	double* user_form;
	double** t_matrix1;
	double** t_matrix2;
	double** r_matrix;
	double* vect_5;
	char grip;
	double** via;
	double*** traj;


public:
	robo_control();
	void zeroPosition();
	void moveJOINT();
	void currentJoints();
	void currentCartesian();
	void moveCart();
	void initJoint();
	void stopRobot();
	void moveGriper();
	void trajectoryPlan();								//KARA
														//done
};

#endif 