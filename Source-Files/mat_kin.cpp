#include "mat_kin.h"

using namespace std;

mat_kin::mat_kin()
{
	//TMULT
	r_matrix = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		r_matrix[i] = new double[WIDTH];
	}

	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			r_matrix[i][j] = 0;
		}
	}

	//TINVERT
	for (int i = 0; i < 3; i++)
	{
		user_form[i] = 0;
	}

	//WHERE
	user_form_ptr = new double[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		user_form_ptr[i] = 0;
	}

	internal_form = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		internal_form[i] = new double[WIDTH];
	}

}
/* Function UTOI:
* Takes the user information of x y,z and theta
* to produce the useable input of the transformation matrix
*/
double** mat_kin::UTOI(double* user_form)
{
	//Retreiving values
	x = user_form[0];
	y = user_form[1];
	z = user_form[2];
	theta = user_form[3] * (PI / 180);

	//cout << x << "\t" << y << "\t" << z << "\t" << theta << endl;
	//cout << endl;

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
double* mat_kin::ITOU(double** internal_form)
{
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
double** mat_kin::TMULT(double** t_matrix1, double** t_matrix2)
{
	//Mutliply the Rotation Matirices together
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < WIDTH; col++) {
			// Multiply the row of A by the column of B to get the row, column of product.
			for (int inner = 0; inner < WIDTH; inner++) {
				r_matrix[row][col] += t_matrix1[row][inner] * t_matrix2[inner][col];
			}
		}
	}

	return r_matrix;
}

double** mat_kin::TINVERT(double** internal_form)
{
	temp = internal_form[0][1];
	internal_form[0][1] = internal_form[1][0];
	internal_form[1][0] = temp;

	temp = internal_form[0][2];
	internal_form[0][2] = internal_form[2][0];
	internal_form[2][0] = temp;

	temp = internal_form[1][2];
	internal_form[1][2] = internal_form[2][1];
	temp = internal_form[2][1];

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

double** mat_kin::WHERE(double* joint)
{

	the1 = joint[0] * (PI / 180);
	the2 = joint[1] * (PI / 180);
	d3 = joint[2];
	the4 = joint[3] * (PI / 180);

	//internal_form[0][0] = cos(the4 - the1 - the2);
	internal_form[0][0] = cos(the1 + the2 - the4);

	//internal_form[0][1] = sin(the4 - the1 - the2);
	internal_form[0][1] = sin(the1 + the2 - the4);

	internal_form[0][2] = 0;
	internal_form[0][3] = 195 * cos(the1) + 142 * cos(the1 + the2);

	//internal_form[1][0] = -sin(the4 - the1 - the2);
	internal_form[1][0] = sin(the1 + the2 - the4);

	//internal_form[1][1] = cos(the4 - the1 - the2);
	internal_form[1][1] = cos(the1 + the2 - the4);

	internal_form[1][2] = 0;
	internal_form[1][3] = 195 * sin(the1) + 142 * sin(the1 + the2);

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

double** mat_kin::SOLVE(double* oldJoints, double** Tmatrix)
{
	double **joints;
	double px = Tmatrix[0][3];
	double py = Tmatrix[1][3];
	double pz = Tmatrix[2][3];
	double r10 = Tmatrix[1][0];
	double r00 = Tmatrix[0][0];



	double joint1[2];
	double joint2[2];
	double joint3;
	double joint4[2];

	double A;
	double r;
	double gamma;

	joints = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		joints[i] = new double[WIDTH];
	}
	joints[3][3] = 1;

	//theta2 = findTheta2(px,py, theta1[0], theta1[1]);	
	A = (px*px + py*py - 142 * 142 - 195 * 195) / (2 * 195 * 142);
	if (A*A > 1) {
		cout << "no solution" << endl;
		joints[3][3] = 0;
	}
	joint2[0] = -atan2(sqrt(1 - A*A), A);
	joint2[1] = -atan2(-sqrt(1 - A*A), A);


	//theta1 = findTheta1(px, py, oldJoints[1]);
	for (int i = 0; i < 2; i++) {
		r = sqrt(pow((195 + 142 * cos(joint2[i])), 2) + pow(142 * sin(joint2[i]), 2));
		gamma = atan2(142 * sin(joint2[i]) / r, (195 + 142 * cos(joint2[i])) / r);
		joint1[i] = atan2(py / r, px / r) - gamma;
	}

	joint3 = -pz - 480;															//Find value of d3
	//joint3 =-195-pz;
	//theta4 = findTheta4(r01, r11, oldJoints[3], joint1[0], joint2[0]);			//Find value of Theta4
	
	joint4[0] = joint1[0] + joint2[0] - atan2(r10, r00);
	joint4[1] = joint1[1] + joint2[1] - atan2(r10, r00);

	joint1[0] = joint1[0] * 180 / PI;
	joint1[1] = joint1[1] * 180 / PI;

	joint2[0] = joint2[0] * 180 / PI;
	joint2[1] = joint2[1] * 180 / PI;

	joint4[0] = (joint4[0] * 180 / PI);
	joint4[1] = (joint4[1] * 180 / PI);


	//** Error Checking **
	if ((joint1[0] > 150 || joint1[0] < -150) && (joint1[1] > 150 || joint1[1] < -150)) {
		cout << "SOLVE_ERROR: joint 1 limit " << endl << endl;
		joints[3][3] = 0;
	}
	if ((joint2[0] > 100 || joint2[0] < -100) && (joint2[1] > 100 || joint2[1] < -100)) {
		cout << "SOLVE_ERROR: joint2[0]  limit" << endl << endl;
		joints[3][3] = 0;
	}
	if (joint3 < -200 || joint3 >-100) {
		cout << "SOLVE_ERROR: Joint 3 limit " << endl << endl;
		joints[3][3] = 0;
	}
	if ((joint4[0] > 160 || joint4[0] < -160) && (joint4[1] >160 || joint4[1] < -160)){
		cout << "SOLVE_ERROR: joint4  limit " << endl << endl;
		joints[3][3] = 0;
	}

	//place results into output array

	joints[0][0] = joint1[0];
	joints[1][0] = joint1[1];
	joints[0][1] = joint2[0];
	joints[1][1] = joint2[1];
	joints[0][2] = joint3;
	joints[0][3] = joint4[0];
	joints[1][3] = joint4[1];

	return joints;
}
/*Function: findTheta1
* Inputs the transformation matrix from {0} to {5}, the previous joint values
* and Ouputs the possible solutions of theta1 and a flag to verfy that there is a soltuion
*/
double* mat_kin::findTheta1(double px, double py, double oldJoint1)
{
	double* theta1;
	double a, b, c, d;

	a = px;
	b = py;
	c = (a*a + b*b + 17861) / 390;
	d = sqrt(b*b - c*c + a*a);

	theta1 = new double[3];
	theta1[2] = 1;

	if ((c + a) == 0) { 
		theta1[0] = 2 * atan2(-a / b, 1);
		theta1[1] = theta1[0];
	}
	else if (a == 0 && c > 0) {
		theta1[0] = PI / 2;
		theta1[1] = theta1[0];
	}
	else if (a == 0 && c < 0) {
		theta1[0] = -PI / 2;
		theta1[1] = theta1[0];
	}
	else if (b == 0) {
		if ((int)c == 0) {
			theta1[0] = oldJoint1;
			theta1[1] = oldJoint1;
			cout << "infinite solutions" << endl;
		}
		else {
			cout << "no solution" << endl;
			theta1[2] = 0;
		}
	}
	else {
		theta1[0] = 2 * atan2((b + d) / (c + a), 1);
		theta1[1] = 2 * atan2((b - d) / (c + a), 1);
	}


	return theta1;
}


/*Function: findTheta2
* Input the transformation matrix from {0} to {5} and the value of theta1
* the function Outputs the two possible values of theta2 and a flag to verfy that they are valid calculation wise (no joint limits are calculated here)
*/
double* mat_kin::findTheta2(double px, double py, double theta1_1, double theta1_2) {

	double C2;	//simply cosine(theta2)
	double* theta2;

	theta2 = new double[3];
	C2 = (cos(theta1_1)*px + py*sin(theta1_1) - 195) / 142;

	if (C2 == 0)
	{
		theta2[0] = PI / 2;
		theta2[1] = theta2[0];
	}
	else if (pow(C2, 2)>1) {
		cout << "ERROR: Joint 2 is not solvable"<<endl; 
	}
	/* else if(C2 == 1) {
	theta2[0] = 0;
	theta2[1] = PI;
	}
	*/
	else if (C2 > 0) {
		theta2[0] = atan2(sqrt((1 - C2*C2)), C2);
		theta2[1] = atan2(-sqrt((1 - C2*C2)), C2);
	}
	else if (C2 < 0) {
		theta2[0] = atan2(sqrt((1 - C2*C2)), C2) + PI;
		theta2[1] = atan2(sqrt((1 - C2*C2)), C2) - PI;

	}

	return theta2;
}


/*Function: findTheta4
* Inputs the transformation matrix from {0} to {5}, the previous joint values, the values of theta1 and theta2
* and Ouputs the possible solutions of theta4 and a flag to verfy that there is a soltuion
*/
double* mat_kin::findTheta4(double r01, double r11, double oldJoint4, double theta1, double theta2) {
	
	double* theta4;
	double a, b, c, d;

	c = cos(theta1)*r01 + r11 * sin(theta1);
	a = sin(theta2);
	b = cos(theta2);
	d = sqrt(b*b - c*c + a*a);

	theta4 = new double[3];
	theta4[2] = 1;


	if ((c + a) == 0) { //cout << "Degenerate case (c + a) == 0"<<endl;
		theta4[0] = 2 * atan2(-a / b, 1);
		theta4[1] = theta4[0];
	}
	else if (a == 0 && c > 0) {
		theta4[0] = PI / 2;
		theta4[1] = theta4[0];
	}
	else if (a == 0 && c < 0) {
		theta4[0] = -PI / 2;
		theta4[1] = theta4[0];
	}
	else if (b == 0) {
		if ((int)c == 0) {
			theta4[0] = oldJoint4;
			theta4[1] = oldJoint4;
			cout << "infinite solutions" << endl;
		}
		else {
			theta4[2] = 0;
		}
	}
	else {
		theta4[0] = 2 * atan2((b + d) / (c + a), 1);
		theta4[1] = 2 * atan2((b - d) / (c + a), 1);
	}

	return theta4;
}

/*Function: printInternalMatrix
* Takes 8x5 matrix
* Then ouputs the result as a tab list

void mat_kin::printInternalMatrix(double** matrix) {

	for (int x = 0; x<8; x++) {
		for (int y = 0; y<=4; y++) {
			cout << matrix[x][y] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;
}
*/
void mat_kin::printMatrix(double** matrix, int height, int width) {

	for (int y = 0; y< height; y++) {
		for (int  x = 0; x< width; x++) {
			cout << setw(15) << matrix[y][x];// << "\t";
		}
		cout << endl;
	}
	//cout <<endl;
}

bool mat_kin::errorFound(double *values, int selection) {

	if (selection==1){		// Check Joint values
		if (values[0] > 150 || values[0] < -150) {
			cout << "ERROR: Joint 1 limit";
			return true;
		}
		else if (values[1] > 100 || values[1] < -100) {
			cout << "ERROR: Joint 2 limit";
			return true;
		}
		else if (values[2] < -200 || values[2]> -100) {
			cout << "ERROR: Joint 3 limit";
			return true;
		}
		else if (values[3] > 160 || values[3] < -160) {
			cout << "ERROR: Joint 4 limit";
			return true;
		}
		else {
			return false;
		}
	}
	else if (selection == 2) {		// Check Joint Velocities
	
		//Revolute joints 1, 2, 4 = [-150, 150] mm / s
		//Prismatic Joint 3 = [-50, 50] mm / s

			if (values[0] > 150 || values[0] < -150) {
				cout << "ERROR: Velocity Joint 1 limit";
				return true;
			}
			else if (values[1] > 150 || values[1] < -150) {
				cout << "ERROR: Velocity Joint 2 limit";
				return true;
			}
			else if (values[2] < 50 || values[2]>-50) {
				cout << "ERROR: Velocity Joint 3 limit";
				return true;
			}
			else if (values[3] >150 || values[3] < -150) {
				cout << "ERROR: Velocity Joint 4 limit";
				return true;
			}
			else {
				return false;
			}
	}
	else if (selection == 3) {		// Check Joint Acceleration
		
		//	Revolute joints 1, 2, 4 = [-600, 600] mm / s^2
		//	Prismatic Joint 3 = [-200, 200] mm / s^2
			if (values[0] > 600 || values[0] < -600) {
				cout << "ERROR: Acceleration Joint 1 limit";
				return true;
			}
			else if (values[1] > 600 || values[1] < -600) {
				cout << "ERROR: Acceleration Joint 2 limit";
				return true;
			}
			else if (values[2] <= -200 || values[2]> 200) {
				cout << "ERROR: Acceleration Joint 3 limit";
				return true;
			}
			else if (values[3] >600 || values[3] < -600) {
				cout << "ERROR: Accelerationy Joint 4 limit";
				return true;
			}
			else {
				return false;
			}
	
	}
	return true;

}
