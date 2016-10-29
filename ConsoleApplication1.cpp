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

int main() {

	double* test_user;
	double** test_matrix;
	double** tmult_matrix;
	double** tinvert_matrix;

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
* Takes the Transformation matrix and outputs the values of x, y and theta
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

