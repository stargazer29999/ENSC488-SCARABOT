#ifndef TRAJ_PLAN
#define TRAJ_PLAN
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>

#include "mat_kin.h"

#define WIDTH 4
#define TIME 999 	// the final time divided by the time resolution NOTE:if 999 or -999 that is a placeholder a value
#define RES 10	// the time resolution
#define FINAL -999	// the final time 
//#define revolMaxVel  [-150 150]
//#define prismMaxVel [-50 50]
#define revolMaxAcc	[-600 600]
#define prismMaxAcc [-200 200]

class traj_plan
{
private:
	double joint[5];
	double*** Output;
	double** spline;
	double** coeff;
	double* coefficient; // this is not the same value a coeff
	double** discreteSpline;
	double h_0, h_1, h_2, h_3;
	double h_4; //time btw. last via and goal?
	double delta0, delta1, delta2, delta3;
	double v1, v2, v3, v4;
	double* tau;
	double deltaT;
	int vi_1;

public:
	double*** discreteTrajectory(double** viaPoints, int numVia);
	void traj_plan::printTrajectory(string filename, double** jointValues);
	double** traj_plan::calcSpline(double* joint, int numVia, int i, double* time);
	double* traj_plan::calcCoeff(double yi, double yi_1, double vi, double vi_1, double hi);
	double** traj_plan::calcDiscreteSpline(double t0, double tf, double* coeff );

};

#endif 
