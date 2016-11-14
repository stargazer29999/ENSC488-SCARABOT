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
#define TIME 30 	//TIME is the final time divided by the time resolution

class traj_plan
{
private:
	double joint[5];
	double*** Output;
	double** spline;


public:
	double*** discreteTrajectory(double** viaPoints, int numVia);
	void traj_plan::printTrajectory(string filename, double** jointValues);
	double** traj_plan::calcSpline(double* joint, int numVia);

};

#endif 