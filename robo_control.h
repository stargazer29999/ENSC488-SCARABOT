#ifndef ROBO_CONTROL
#define ROBO_CONTROL
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <ctime>
#include <queue>
#include <vector>

#include "mat_kin.h"
#include "ensc-488.h"


#define HEIGHT 4
#define WIDTH 4
#define PI 3.14159265359

class robo_control
{
private:
	mat_kin cmd;
	//traj_plan cmd2;

	JOINT* q0;
	JOINT* q1;



	double j1, j2, j3, j4;
	
	vector<double> internal_form;
	vector<double> user_form;
	vector<double> t_matrix1;
	vector<double> t_matrix2;
	vector<double> r_matrix;
	vector<double> vect_5;
	vector<double> via;
	vector<double> traj;
	vector<double> traj_joint0;
	vector<double> traj_joint1;
	vector<double> traj_joint2;
	vector<double> traj_joint3;
	vector<double> vectQ0;
	vector<double> vectQ1;

	double x, y, z, theta;
	char grip;

	/*

	double** internal_form;
	double* user_form;
	double** t_matrix1;
	double** t_matrix2;
	double** r_matrix;
	double* vect_5;
	double** via;
	double*** traj;
	int ii;
	double ** traj_joint0;
	double ** traj_joint1;
	double ** traj_joint2;
	double ** traj_joint3;
	queue<double**> *sampleQueue;
	*/
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
	void moveWithVelAcc(JOINT & q1, JOINT & q2, JOINT & q3);
	void RESETROBOT();
														//done
};

#endif 
