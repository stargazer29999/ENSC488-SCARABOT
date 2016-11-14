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
#define RES -999	// the time resolution
#define FINAL -999	// the final time 

class traj_plan
{
private:
	double joint[5];
	double*** Output;
	double** spline;
	double* coeff;
	double** discreteSpline;

public:
	double*** discreteTrajectory(double** viaPoints, int numVia);
	void traj_plan::printTrajectory(string filename, double** jointValues);
	double** traj_plan::calcSpline(double* joint, int numVia);
	double* traj_plan::calcCoeff(double yi, double yi_1, double vi, double vi_1, double hi);
	double** traj_plan::calcDiscreteSpline(double t0, double tf, double timeReso, double* coeff);

};

#endif 
