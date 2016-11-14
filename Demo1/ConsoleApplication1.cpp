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
double** WHERE(double* joint);
double* SOLVE(double* oldJoints, double** Tmatrix);

//double *findTheta1(double **Tmatrix, double* oldJoints);
double *findTheta1(double px,double py, double* oldJoints);
//double* findTheta2(double **Tmatrix, double theta1);
double* findTheta2(double Tmatrix, double theta1);
//double* findTheta4(double **Tmatrix, double* oldJoints, double theta1, double theta2);
double* findTheta4(double r01, double r11, double* oldJoints, double theta1, double theta2);
void printInternalMatrix(double **matrix);

int main() {

	double* test_user;
	double** test_matrix;
	double** tmult_matrix;
	double** tinvert_matrix;
	double* test_joint;
	double* test_location;
	double* newjoints;

	test_user = new double[HEIGHT];
	test_matrix = new double*[HEIGHT];

	for (int i = 0; i < HEIGHT; i++) {
		test_user[i] = 0;
		test_matrix[i] = new double[WIDTH];
	}

	cout << "** all input joint values are between -180 and 180*" << endl;

/*
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


	//Test 2: ITOU
	cout << "Test 2: ITOU"<< endl;
	test_user = ITOU(test_matrix);
	cout << endl;

	for (int x = 0; x<HEIGHT; x++) {
		cout << test_user[x] << "\t";
	}
	cout << endl<< endl;

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
	*/
	//Test 5: WHERE
	test_joint = new double[HEIGHT];
	test_joint[0] = 0;
	test_joint[1] = 0;
	test_joint[2] = -150;
	test_joint[3] = 0;

	cout << "Test 5: WHERE" << endl;
	test_matrix = WHERE(test_joint);
	test_location = ITOU(test_matrix);
	
	for (int x = 0; x<HEIGHT; x++) {
			cout << test_location[x]<< "\t";
	}
	cout << endl << endl;

	
	//Test 6: SOLVE
	newjoints=new double[HEIGHT];

	cout << "Test 6: SOLVE" << endl;
	newjoints = SOLVE(test_joint, test_matrix);

	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl << endl;

	test_user[0] = 25;		//j1
	test_user[1] = 60;		// j2
	test_user[2] = -101  ;		// j3
	test_user[3] = 35;		// j4

	test_matrix = WHERE(test_user);
	for (int x = 0; x<HEIGHT; x++) {
		for (int y = 0; y<WIDTH; y++) {
			cout << test_matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;

	newjoints = SOLVE(test_joint, test_matrix);

	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl << endl;

	test_user[0] = 140;		//j1
	test_user[1] = -95;		// j2
	test_user[2] = -101;		// j3
	test_user[3] = 155;		// j4
	
	test_matrix = WHERE(test_user);
	for (int x = 0; x<HEIGHT; x++) {
		for (int y = 0; y<WIDTH; y++) {
			cout << test_matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;

	newjoints = SOLVE(test_joint, test_matrix);

	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl << endl;


	test_user[0] = 0;		//j1
	test_user[1] = 85;		// j2
	test_user[2] = -101;		// j3
	test_user[3] = 0;		// j4

	test_matrix = WHERE(test_user);
	for (int x = 0; x<HEIGHT; x++) {
		for (int y = 0; y<WIDTH; y++) {
			cout << test_matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;

	newjoints = SOLVE(test_joint, test_matrix);

	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl << endl;



	cout << "Test 6.1: SOLVE for  no solution" << endl;
	//test for inavlid joint varaibles

	//checked for invalid x&y ->passed
	//checked for an invalid z/joint 3 value -> passed
	test_user[0] = 0;		//x
	test_user[1] = 336;		// y
	test_user[2] = -200;		// z value
	test_user[3] = 0;		// theta

	test_matrix = UTOI(test_user);
	newjoints = SOLVE(test_joint, test_matrix);
	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl;

	//check for invalid joint 4 values -> passed   
	test_user[0] = -220.3787;		//x
	test_user[1] = -2.6589;			// y
	test_user[2] = -300.0000;		// z value
	test_user[3] = 140.0000;		// theta

	test_matrix = UTOI(test_user);
	newjoints = SOLVE(test_joint, test_matrix);
	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl;
	//test with an invalid joint 1 value	 -> passed    
	test_user[0] = -316.6764;		//x
	test_user[1] = -66.6939;		// y
	test_user[2] = -355.0000;		// z value
	test_user[3] = 50.0000;		// theta


	test_matrix = UTOI(test_user);
	newjoints = SOLVE(test_joint, test_matrix);
	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl;
	//test with an invalid joint 2 value ->passed
	test_user[0] = 94.5908;		//x
	test_user[1] = 0;		// y
	test_user[2] = -355.0000;		// z value
	test_user[3] =125.0000;		// theta

	cout <<endl<< " input x,y,z, theta gives a transformation matrix" << endl;
	test_matrix = UTOI(test_user);

	for (int x = 0; x < HEIGHT; x++) {	//print output
		for (int y = 0; y < WIDTH; y++) {
			cout << test_matrix[x][y] << "\t";
		}
		cout << endl;
	}	

	newjoints = SOLVE(test_joint, test_matrix);
	cout << "the new joint values are:" << endl;
	for (int x = 0; x<HEIGHT; x++) {
		cout << newjoints[x] << "\t";
	}
	cout << endl << endl;
	
	

//PUT NOTHING HERE -BREAKPOINT
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
			//cout << ""<<endl;
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
			//cout << point[row] << ""<<endl;
		}
		//cout << ""<<endl;
	for (int row = 0; row < 3; row++) {
		internal_form[row][3] = -1 * point[row];
	}

	return internal_form;
}


//Inputs the joint values and ouputs the location of the last frame as a transformation matrix
double** WHERE(double* joint) {

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

	//Error code is the return zero correct?
	if (210>=joint[0]> 150 || joint[0] < -150) {
		cout << "ERROR: Joint 1 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (joint[1] > 100 || joint[1] < -100) {
		cout << "ERROR: Joint 2 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (joint[2]<=-200 || joint[2]>-100) {
		cout << "ERROR: Joint 3 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (joint[3] > 160 || joint[3] <=-160) {
		cout << "ERROR: Joint 4 limit\n CHANGE code in GUI to Exit Program";
	}

	else {
	
		theta1 = joint[0] * (PI / 180);
		theta2 = joint[1] * (PI / 180);
		d3 = joint[2];
		theta4 = joint[3] * (PI / 180);

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
		//user_form = ITOU(internal_form);

		return internal_form;
	}

}


/*Function: SOLVE
* Inputs the pointers to the previous joint values and the transfromation matrix from {0} to {5}
* the Ouput is a 4x1 vector of the resultant joint values
*/
double* SOLVE(double* oldJoints, double** Tmatrix) {

	double* joints;
	double* theta1;
	double* theta2;
	double* theta4;

	double d3 = Tmatrix[2][3];
	double px = Tmatrix[0][3];
	double py = Tmatrix[1][3];
	double r01 = Tmatrix[0][1];
	double r11 = Tmatrix[1][1];

//	theta1=new double[2];

	//Make a 5x4 double to store all possible solutions or and an index to tell if value is valid or not (1==valid, 0==inavlid)
	double** internal_joints;
	//Allocate memory
	internal_joints = new double*[8];
	for (int i = 0; i < 8; i++) {
		internal_joints[i] = new double[5];
	}

	//Column 0 is validity index, Column 1 is joint 1, column 2 is joint 2, etc
	
	for (int i = 0; i < 8; i++) { //Intialize all solutions as valid
		internal_joints[i][0] = 1;
	}
//	cout << "Loaded with ones" << endl;
	//printInternalMatrix(internal_joints);

	
	//************************Find value of d3**********************************

	for (int i = 0; i < 8; i++) {
		internal_joints[i][3]= -d3 - 480;

		//error checking
		if (internal_joints[i][3]<=-200 || internal_joints[i][3]>-100) {
			internal_joints[i][0]= 0;
			//cout << "SOLVE_ERROR: Joint 3 limit "<<endl;
		}
	}
	//cout << "found d3" << endl;
//	printInternalMatrix(internal_joints);

	//******************Find value of Theta1********************************
	//Do calcualtions
	 int i = 0;
	while (i < 8) {
		theta1 = findTheta1(px,py, oldJoints);
		internal_joints[i][1] = theta1[0] * 180 / PI;
		//Error Checking
		if (internal_joints[i][1] >= 150 || internal_joints[i][1] <= -150 || theta1[2] == 0) {
			internal_joints[i][0] = 0;
			//cout << "SOLVE_ERROR: Joint 4 limit reached\t invalid Tmatrix"<<endl;
		}
		//printInternalMatrix(internal_joints);
		i++;
		internal_joints[i][1] = theta1[1] * 180 / PI;
		if (internal_joints[i][1] >= 150 || internal_joints[i][1] <= -150 || theta1[2] == 0) {
			internal_joints[i][0] = 0;
			//cout << "SOLVE_ERROR: Joint 4 limit reached"<<endl;
		}
		i++;
		//printInternalMatrix(internal_joints);
	};
	


/*	if (Tmatrix[0][2] == 0) { //Degen case:1
		theta1 = 0;
		theta1_ = PI;
	}
	else if (Tmatrix[1][2] == 0) { //Degen case:2
		theta1 = PI/2;
		theta1_ = theta1 +PI;
	}
	else {
		theta1 = 2 * atan2((Tmatrix[1][2] + sqrt(pow(Tmatrix[1][2], 2) + pow(Tmatrix[0][2], 2))) / Tmatrix[0][2], 1);
		theta1_ = 2 * atan2((Tmatrix[1][2] - sqrt(pow(Tmatrix[1][2], 2) + pow(Tmatrix[0][2], 2))) / Tmatrix[0][2], 1);
	}

	//Store theta 1 in Internal_matrix and error check
	for (int i = 0; i < 8; i++) {
		if (i < 4) { 
			internal_joints[i][1] = theta1*180/PI;
		}
		else {
			internal_joints[i][1] = theta1_*180/PI;
		}
		//Error Checking
		if (internal_joints[i][1] >= 150 ||internal_joints[i][1]<=-150) {
			internal_joints[i][0] = 0;
			//cout << "SOLVE_ERROR: Joint 1 limit reached\t invalid Tmatrix"<<endl;
		}
	}
	*/
	//cout << "Found theta1" << endl;
	//printInternalMatrix(internal_joints);

	//******************Find value of Theta2********************************
//	int i = 0;
	i = 0;
	while (i < 8) {
		theta2 = findTheta2(px, internal_joints[i][1]*PI/180);
		internal_joints[i][2]= theta2[0] * 180/PI;
		//printInternalMatrix(internal_joints);

	//Error Checking
		if (internal_joints[i][2] >= 100 || internal_joints[i][2] <= -100 || theta2[2] == 0) {
			internal_joints[i][0] = 0;
			//cout << "SOLVE_ERROR: Joint 2 limit reached\t invalid Tmatrix"<<endl;
		}
		i++;
		internal_joints[i][2] = theta2[1] * 180/PI;
		//printInternalMatrix(internal_joints);
	//Error Checking
		if (internal_joints[i][2] >= 100 || internal_joints[i][2] <= -100|| theta2[2]==0) {
			internal_joints[i][0] = 0;
		//	cout << "SOLVE_ERROR: Joint 2 limit reached\t invalid Tmatrix "<<endl;
		}
		i++;
	};

	//cout << "Found theta2" << endl;
	//printInternalMatrix(internal_joints);

	//****************** Find value of Theta4 ******************
	i = 0;

	while(i < 8) {
		theta4=findTheta4(r01, r11,oldJoints, internal_joints[i][1] * PI / 180, internal_joints[i][2] * PI / 180);
		internal_joints[i][4] = theta4[0] * 180/PI;
		//Error Checking
		if (internal_joints[i][4] >=160 || internal_joints[i][4] <= -160 || theta4[2] == 0) {
			internal_joints[i][0]= 0;
			//cout << "SOLVE_ERROR: Joint 4 limit reached\t invalid Tmatrix"<<endl;
		}
		i++;
		internal_joints[i][4] = theta4[1] * 180/PI;
			if (internal_joints[i][4] >= 160 || internal_joints[i][4] <= -160 || theta4[2] == 0) {
				internal_joints[i][0]= 0;
				//cout << "SOLVE_ERROR: Joint 4 limit reached"<<endl;
			}
		i++;
	};

	//cout << "Found theta4" << endl;
//	printInternalMatrix(internal_joints);

	
	//**********Find the shortest distance if soltuon is valid********************
	double minArray[8] = { 0,0,0,0,0,0,0,0 };
	double min;
	int index;
	bool noSolution = true;

	//sum the metric distances
	for (int i = 0; i < 8; i++) {
			for (int j = 1; j < 4; j++) {
				minArray[i] += abs(internal_joints[i][j] - oldJoints[j]);
			}
	}
	//find the smallest value
	min = minArray[0];
	index = 0;

	for (int i = 0; i < 8; i++) {
		if (internal_joints[i][0] != 0 && min>minArray[i]) {
			//find index of shortest distance
			min = minArray[i];
			index = i;
			noSolution = false;
		}
	}

	if (noSolution && internal_joints[index][0] == 0) {
		cout << "** No Solution found do not set Device to run **"<<endl;
	}

	//place results into output array
	joints = new double[HEIGHT];

		for (int i = 0; i < HEIGHT; i++) {
			joints[i] = internal_joints[index][i+1];
		}

	return joints;
}

/*Function: findTheta1
* Inputs the transformation matrix from {0} to {5}, the previous joint values
* and Ouputs the possible solutions of theta1 and a flag to verfy that there is a soltuion
*/
double *findTheta1(double px,double py, double* oldJoints) {

	double* theta1;
	double a, b, c;

	a = px;
	b = py;
	c = (pow(px,2) + pow(py, 2) + 17861) / 390;
	
	theta1 = new double[3];
	theta1[2] = 1;

	if (a == 0) {
		if (b == 0) {
			if (c == 0) {
				//no sln
				theta1[2] = 0;
			}
			else {
				//infinite slns
				theta1[0] = oldJoints[1];
				theta1[1] = oldJoints[1];
			}
		}
		else {
			if (c == 0) {
				theta1[0] = 0;
				theta1[1] = PI;
			}
			else {
				if (c / b > 1) {
					//no solution
					theta1[2] = 0;
				}
				else {
					//s1=c/b
					theta1[0] = atan2(c / b, sqrt(1 - (c / b)*(c / b)));
					theta1[1] = atan2(c / b, -sqrt(1 - (c / b)*(c / b)));
				}
			}
		}
	}
	else {
		if (b == 0) {
			if (c == 0) {
				theta1[0] = PI / 2;
				theta1[1] = -PI / 2;
			}
			else {
				if (c / a > 1) {
					//no solution
					theta1[2] = 0;
				}
				else {
					//c1=c/a
					theta1[0] = atan2(sqrt(1 - (c / a)*(c / a)), (c / a));
					theta1[1] = atan2(-sqrt(1 - (c / a)*(c / a)), (c / a));
				}
			}
		}
		else {
			if (c == 0) {
				//	ac1+bs1=0
				theta1[0] = 2 * atan2((b + sqrt(b*b + a*a)) / a, 1);
				theta1[1] = 2 * atan2((b - sqrt(b*b + a*a)) / a, 1);
			}
			else {
				//	ac1+bs1=c
				theta1[0] = 2 * atan2((b + sqrt(b*b - c*c + a*a)) / (c + a), 1);
				theta1[1] = 2 * atan2((b - sqrt(b*b - c*c + a*a)) / (c + a), 1);
			}
		}
	}
	return theta1;
}


/*Function: findTheta2
* Input the transformation matrix from {0} to {5} and the value of theta1
* the function Outputs the two possible values of theta2 and a flag to verfy that they are valid calculation wise (no joint limits are calculated here)
*/
double* findTheta2(double temp, double theta1) {
	
	double c12;	//simply cosine(theta2)
	double* theta2;

	
	theta2 = new double[3];
	theta2[2] = 1;	//assign intial valid solution
	
	c12 = (temp-(195 * cos(theta1))) / 142;
	if (c12*c12 > 1) {
		//no solution
		theta2[2] = 0;
	}
	else {
		theta2[0] = atan2(sqrt(1 - c12*c12), c12)-theta1;
		theta2[1] = atan2(-sqrt(1 - c12*c12), c12)-theta1;
	}
	
	return theta2;
}

/*Function: findTheta4
* Inputs the transformation matrix from {0} to {5}, the previous joint values, the values of theta1 and theta2 
* and Ouputs the possible solutions of theta4 and a flag to verfy that there is a soltuion
*/
double *findTheta4(double r01,double r11, double* oldJoints, double theta1, double theta2) {
	
	double* theta4;
	double a, b, c;
	
	c = cos(theta1)*r01 + r11 * sin(theta1);
	a = sin(theta2);
	b = cos(theta2);

	theta4 = new double[3];
	theta4[2] = 1;

	if (a == 0) {
		if (c == 0) {
			theta4[0] = 0;
			theta4[1] = PI;
		}
		else {
			if (c / b > 1) {
				//no solution
			}
			else {
				//s4=c/b
				theta4[0] = atan2(c / b, sqrt(1 - (c / b)*(c / b)));
				theta4[1] = atan2(c / b, -sqrt(1 - (c / b)*(c / b)));
			}
		}
	}
	else {
		if (b == 0) {
			if (c == 0) {
				theta4[0] = PI / 2;
				theta4[1] = -PI / 2;
			}
			else {
				if (a / c > 0) {
					//no solution
					theta4[2] = 0;
				}
				else {
					//c4=c/a
					theta4[0] = atan2(sqrt(1 - (c / a)*(c / a)), (c / a));
					theta4[1] = atan2(-sqrt(1 - (c / a)*(c / a)), (c / a));
				}
			}
		}
		else {
			if (c == 0) {
				//0=a c4+ b s4
				theta4[0] = 2 * atan2((b + sqrt(b*b + a*a)) / a, 1);
				theta4[1] = 2 * atan2((b - sqrt(b*b + a*a)) / a, 1);
			}
			else {
				//c=a c4+ b s4
				theta4[0] = 2 * atan2((b + sqrt(b*b - c*c + a*a)) / (c + a), 1);
				theta4[1] = 2 * atan2((b - sqrt(b*b - c*c + a*a)) / (c + a), 1);
			}
		}
	}

	return theta4;
}

/*Function: printInternalMatrix
* Takes 8x5 matrix 
* Then ouputs the result as a tab list
*/
void printInternalMatrix(double** matrix) {

	for (int x = 0; x<8; x++) {
		for (int y = 0; y<5; y++) {
			cout << matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;
}
