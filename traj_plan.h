#ifndef TRAJ_PLAN
#define TRAJ_PLAN
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#include "mat_kin.h"

#define WIDTH 4
#define RES 100	// the time resolution (ms)

class traj_plan
{
private:
	vector<double> spline, coeff, segment1, segment2, segment3, segment4, temp, coefficient, tau, time, joint, Output;

	double t;
	int ii, via0, via1, via2, via3; points;

	double h_0, h_1, h_2, h_3;
	double h_4; //time btw. last via and goal?
	double delta0, delta1, delta2, delta3;
	double v1, v2, v3, v4;
	double deltaT;
	double total_time;

	int vi_1;
	int steps;

public:
	traj_plan();
	vector<double> discreteTrajectory(vector<double>, int numVia, int numPoints, int jointNUM);
	void traj_plan::printTrajectory(string filename, vector<double> jointValues, int steps);
	vector<double> traj_plan::calcSpline(vector<double> joint, int numVia, /*int i,*/ vector<double> *time, int numPoints);
	vector<double> traj_plan::calcCoeff(double yi, double yi_1, double vi, double vi_1, double hi);
	vector<double> traj_plan::calcDiscreteSpline(double t0, double tf, vector<double> coeff);
	void traj_plan::appPrintTrajectory(string filename, double input);
	//void traj_plan::printResults(double*** jointValues, int steps, int i, int j, int tempCounter);
	//int traj_plan::timeSampling()
	void traj_plan::printM(vector<double> matrix, int height, int width); //Kara: please remove once code is complete
	int traj_plan::numofPoints(double t1, double t2, double t3, double t4);
};

#endif 
