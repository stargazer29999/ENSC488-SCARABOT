#ifndef MIN_KIN
#define MIN_KIN

#include <cmath>
#include <string>
#include <iostream>
#include <vector>

#include <iomanip>
//#include  "ensc-488.h"

#define HEIGHT 4
#define WIDTH 4
#define PI 3.14159265359

using namespace std;

class mat_kin
{
private:
	vector<double> r_matrix, point, user_form, internal_form, joints, theta1, theta2, theta4, internal_joints, errJoint1, errJoint2;
	double x, y, z, theta, the1, the2, the4, d3, a, b, c, c12;

	/*
	double** r_matrix;
	
	double px, py, r01, r11;
	double point[3];
	double user_form[3];
	double* user_form_ptr;
	double** internal_form;
	double temp;
	double* joints;
	double* theta1;
	double* theta2;
	double* theta4;
	double** internal_joints;
	*/
public:
	mat_kin();
	vector<double> UTOI(vector<double> user_form);
	vector<double> ITOU(vector<double>internal_form);
	vector<double> TMULT(vector<double> t_matrix1, vector<double> t_matrix2);
	vector<double> TINVERT(vector<double> internal_form);
	vector<double> WHERE(vector<double> joints);
	vector<double> SOLVE(vector<double> oldJoints, vector<double> Tmatrix);
	//double* findTheta1(double px, double py, double oldJoint1);
	//double* findTheta2(double px, double py, double theta1_1, double theta1_2);
	//double* findTheta4(double r01, double r11, double oldJoint4, double theta1, double theta2);
	//void printInternalMatrix(double **matrix);
	void printMatrix(vector<double> matrix, int height, int width);
	bool errorFound(vector<double> values, int selection);

};

#endif
