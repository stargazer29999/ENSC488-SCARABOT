// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

/*
* Programming Exercise 2 (Part 2)
* Problem 1
*
* Class Description: Bidirectional convertor btw. user friendly and machine friendly code for a 2x2 rotation matrix and 2x1 position matrix
* Author: CM
* Date: Oct. 24th. 2016
*/

#include "stdafx.h"
#include <vector>
#include <iostream>
#include <cmath>


using namespace std;

#define HEIGHT 4
#define WIDTH 4
#define PI 3.14159265359

/*
* Reference: http://www.cplusplus.com/forum/articles/7459/
* Reference (TMULTI): https://msdn.microsoft.com/en-us/library/hh873134.aspx
*/

// function declarations
double** UTOI(double* user_form);
double* ITOU(double** internal_form);
double** TMULT(double** form1, double** form2);
double** TINVERT(double** internal_form);
double* WHERE(double* joint);
double* SOLVE(double** Tmatrix);

int main() {

	double* test_user;
	double** test_matrix;
	double** tmult_matrix;
	double** tinvert_matrix;
	double* test_joint;
	double* test_location;


	test_user = new double[HEIGHT];
	test_matrix = new double*[HEIGHT];

	for (int i = 0; i < HEIGHT; i++) {
		test_user[i] = 0;
		test_matrix[i] = new double[WIDTH];
	}


	//Test 1: UTOI
	cout << "Test 1: UTOI" << endl;
	test_user[0] = 2;
	test_user[1] = 3;
	test_user[2] = 4;
	test_user[3] = 90;

	test_matrix = UTOI(test_user);

	for (int x = 0; x<HEIGHT; x++) {
		for (int y = 0; y<WIDTH; y++) {
			cout << test_matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;
/*

	//Test 2: ITOU
	cout << "Test 2: ITOU"<< endl;
	test_user = ITOU(test_matrix);

	cout << endl;

	for (int x = 0; x<HEIGHT; x++) {
		cout << test_user[x] << "\t";
	}

	cout << endl<< endl;

	*/
	//Test 3: TMULT
	cout << "Test 3: TMULT" << endl;
	tmult_matrix=TMULT(test_matrix, test_matrix);
	for (int x = 0; x<HEIGHT; x++) {
		for (int y = 0; y<WIDTH; y++) {
			cout << tmult_matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl<< endl;



	//Test 4: TINVERT
	cout << "Test 4: TINVERT" << endl;
	tinvert_matrix = TINVERT(test_matrix);
		for (int x = 0; x<HEIGHT; x++) {
			for (int y = 0; y<WIDTH; y++) {
				cout << tinvert_matrix[x][y] << "\t";
			}
			cout << endl;
		}
	cout << endl<< endl;

	//Test 5: WHERE
	test_joint = new double[HEIGHT];
	test_joint[0] = 0;
	test_joint[1] = 0;
	test_joint[2] = -150;
	test_joint[3] = 0;

	
	cout << "Test 5: KIN" << endl;
	test_location = WHERE(test_joint);
	for (int x = 0; x<HEIGHT; x++) {
			cout << test_location[x]<< "\t";
	}
	cout << endl << endl;


	return 0;
}


/* Function UTOI:
* Takes the user information of x y,z and theta
* to produce the useable input of the transformation matrix
*/
double** UTOI(double* user_form) {

	double** internal_form;
	double x, y,z, theta;

	//Allocate memory
	internal_form = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		internal_form[i] = new double[WIDTH];
	}

	//Retreiving values
	x = user_form[0];
	y = user_form[1];
	z = user_form[2];
	theta = user_form[3] * (PI / 180);

	cout << x << "\t" << y << "\t" << z << "\t" << theta << endl;
	cout << endl;

	//Assign values
	//first row
	internal_form[0][0] = cos(theta);
	internal_form[0][1] = -sin(theta);
	internal_form[0][2] = 0;
	internal_form[0][3] = x;

	//second row
	internal_form[1][0] = sin(theta);
	internal_form[1][1] = cos(theta);
	internal_form[1][2] = 0;
	internal_form[1][3] = y;

	//third row
	internal_form[2][0] = 0;
	internal_form[2][1] = 0;
	internal_form[2][2] = 1;
	internal_form[2][3] = z;

	//fourth
	internal_form[3][0] = 0;
	internal_form[3][1] = 0;
	internal_form[3][2] = 0;
	internal_form[3][3] = 1;

	return internal_form;
}

/* Function: ITOU
* Takes the Transformation matrix and outputs the values of x, y,z and theta
*/
double* ITOU(double** internal_form) {
	double* user_form;

	user_form = new double[HEIGHT];

	for (int i = 0; i < HEIGHT; i++) {
		user_form[i] = 0;
	}

	user_form[0] = internal_form[0][3];
	user_form[1] = internal_form[1][3];
	user_form[2] = internal_form[2][3];
	user_form[3] = acos(internal_form[0][0])*(180 / PI);

	return user_form;
}

/* Function: TMULT
* Takes two transformation matrices of 4x4 and multiplies them together
* Then ouputs the result as a matrix
*/
double** TMULT(double** form1, double** form2) {


	double** form3;

	//Allocate memory
	form3 = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		form3[i] = new double[WIDTH];
	}
	//intitalize to zero
	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			form3[i][j] = 0;
		}
	}

	//Mutliply the Rotation Matirices together
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < WIDTH; col++) {
				// Multiply the row of A by the column of B to get the row, column of product.
				for (int inner = 0; inner < WIDTH; inner++) {
					form3[row][col] += form1[row][inner] * form2[inner][col];
				}
				//cout << form3[row][col] << "\t";
			}
			//cout << "\n";
		}

	return form3;
}


/* Function: TINVERT
* Takes one matrix and inverts the result
* Then ouputs the result as a matrix
*/
double** TINVERT(double** internal_form) {

	double point[3] = { 0,0,0 };
	double temp;

	//Assign values
	//first row
	temp = internal_form[0][1];
	internal_form[0][1] = internal_form[1][0];
	internal_form[1][0] = temp;

	temp = internal_form[0][2];
	internal_form[0][2] = internal_form[2][0];
	internal_form[2][0] = temp;

	temp = internal_form[1][2];
	internal_form[1][2] = internal_form[2][1];
	temp = internal_form[2][1];

	//point = new double [HEIGHT];


	//Mutliply the Rotation Matirices and the Orign Vector
	for (int row = 0; row < 3; row++) {
			// Multiply the row of A by the column of B to get the row, column of product.
			for (int inner = 0; inner < 3; inner++) {
				point[row] += internal_form[row][inner] * internal_form[inner][3];

			}
			//cout << point[row] << "\n";
		}
		//cout << "\n";
	for (int row = 0; row < 3; row++) {
		internal_form[row][3] = -1 * point[row];
	}

	return internal_form;
}


//Inputs the joint values and ouputs the location of the last frame as x,y,z, theta
double* WHERE(double* joint) {

	double** internal_form;
	double theta1, theta2, d3, theta4;
	double* user_form;

	user_form = new double[HEIGHT];

	for (int i = 0; i < HEIGHT; i++) {
		user_form[i] = 0;
	}

	//Allocate memory
	internal_form = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		internal_form[i] = new double[WIDTH];
	}

	theta1 = joint[0] * (PI / 180);
	theta2 = joint[1] * (PI / 180);
	d3 = joint[2];
	theta4 = joint[3] * (PI / 180);

	//Error code
	//											is the return zero correct?
	if (theta1 > 150 || theta1 < -150) {
		cout << "ERROR: Joint 1 limit\n CHANGE code in GUI to Exit Program";
		
	}
	else if (theta2 > 100 || theta2 < -100) {
		cout << "ERROR: Joint 2 limit\n CHANGE code in GUI to Exit Program";
		
	}
	else if (d3<-200 || d3>-100) {
		cout << "ERROR: Joint 3 limit\n CHANGE code in GUI to Exit Program";
		
	}	
	else if (theta4 > 160 || theta4 <-160) {
		cout << "ERROR: Joint 4 limit\n CHANGE code in GUI to Exit Program";
		
	}
	else {
	
		internal_form[0][0] = sin(theta4)*sin(theta1 + theta2) - cos(theta4)*cos(theta1 + theta2);
		internal_form[0][1] = sin(theta1 + theta2 + theta4);
		internal_form[0][2] = 0;
		internal_form[0][3] = 195 * cos(theta1) + 142 * cos(theta1 + theta2);

		internal_form[1][0] = -sin(theta1 + theta2 + theta4);
		internal_form[1][1] = sin(theta4)*sin(theta1 + theta2) - cos(theta4)*cos(theta1 + theta2);
		internal_form[1][2] = 0;
		internal_form[1][3] = 195 * sin(theta1) + 142 * sin(theta1*theta2);

		internal_form[2][0] = 0;
		internal_form[2][1] = 0;
		internal_form[2][2] = -1;
		internal_form[2][3] = -d3 - 480;

		internal_form[3][0] = 0;
		internal_form[3][1] = 0;
		internal_form[3][2] = 0;
		internal_form[3][3] = 1;

		//Transform to user_form;
		user_form = ITOU(internal_form);

		return user_form;
	}

}


//Inputs the Transfer Matrix and ouputs the joint varaibles
double* SOLVE(double** Tmatrix) {

	double* joints;
	double a, b, c;
	double theta1, theta1_, theta2, theta2_, theta2_1, theta2_2, theta4, theta4_;

	joints = new double[HEIGHT];

	//Find value of d3
	joints[2] = -Tmatrix[2][3] - 480;


	//Find value of Theta1
	if (Tmatrix[0][2] == 0 && Tmatrix[1][2] == 0) {
		theta1 = atan2(0, 1);
		theta1_ = atan2(0, 1)+PI; //??
	}
	else {
		theta1 = atan2(Tmatrix[0][2] / Tmatrix[1][2], 1);
		theta1_ = atan2(-Tmatrix[0][2] / Tmatrix[1][2], -1) + PI;
	}

	//Find value of Theta2
	double C2;	//simply cosine(theta2)
	C2 = cos(theta1)*Tmatrix[0][3] / 142 + Tmatrix[1][3] * sin(theta1) / 142 - 195 / 142;

	if ((int)C2 == 1) {
		theta2 = atan2(0, 1);
		theta2_ = atan2(0, 1) + PI;

	}else if (pow(C2, 2)>1) {			///Kara: get someone to check this please

		cout << "ERROR: Joint 2 is not solvable No solution \n CHANGE code in GUI to Exit Program";

	}else {
		theta2 = atan2(sqrt((1 - pow(C2 ,2))), C2);
		theta2_ = atan2(-sqrt((1- pow(C2,2))), C2);
	}

	//Ge thet Second set of possible avlues for theta2
	C2= cos(theta1_)*Tmatrix[0][3] / 142 + Tmatrix[1][3] * sin(theta1_) / 142 - 195 / 142;
	if ((int)C2 == 1) {
		theta2_1 = atan2(0, 1);
		theta2_2 = atan2(0, 1) + PI;

	}else if(pow(C2, 2)>1){			///Kara: get someone to check this please

		cout << "ERROR: Joint 2 is not solvable No solution \n CHANGE code in GUI to Exit Program";

	}else {
		theta2_1 = atan2(sqrt((1 - pow(C2, 2))), C2);
		theta2_2 = atan2(sqrt((1 - pow(C2, 2))), C2);
	}


	//Find value of Theta4
	findTheta4(Tmatrix, theta1, theta2);


	//Error Code


	//Error Code for Theta1
	theta1 = theta1* (180 / PI);

	if (theta1 > 100 || theta1 < -100) {
		cout << "ERROR: Joint 1 limit\n CHANGE code in GUI to ___";

	}
	if (theta1_ > 100 || theta1_ < -100) {
		cout << "ERROR: Joint 1 limit\n CHANGE code in GUI to ___";

	}
	if (joints[2]<-200 || joints[2]>-100) {
		cout << "ERROR: Joint 2 limit\n CHANGE code in GUI to ___";
	}

	//Find the shortest distance


	return joints;
}

double *findTheta4(double **Tmatrix, double theta1, double theta2) {
	double* theta4;
	theta4 = new double[2];

	double a, b, c;
	c = cos(theta1)*Tmatrix[0][1] + Tmatrix[1][1] * sin(theta1);
	a = sin(theta2);
	b = cos(theta2);
	if ((char)(c + a) == 0) {
		cout << "Degenerate case\n";
		theta4[0] = 2 * atan2(-a / b, 1);
		//theta4[1]= 2 * atan2(-a / b, 1);
		cout << "apply for both theta2 and theta2** for a total of 2 cases";
	}
	else if ((char)b == 0) {
		cout << "Degenerate case\n";
		if ((char)c == 0)
			cout << "infinite solutions\n";
		else
			cout << "no solution\n";

	}

	else {
		theta4[0] = atan2((b + sqrt(b*b - c*c + a*a)) / (c + a), 1);
		theta4[1] = atan2((b - sqrt(b*b - c*c + a*a)) / (c + a), 1);
		cout << "apply for both theta2 and theta2** for a total of four cases";
	}

	return theta4;
}

