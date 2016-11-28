#include "mat_kin.h"

using namespace std;

mat_kin::mat_kin()
{
	vector<double> r_matrix(16);
	vector<double> internal_form(16);
	vector<double> user_form(4);
}

/* Function UTOI:
* Takes the user information of x y,z and theta
* to produce the useable input of the transformation matrix
*/
vector<double> mat_kin::UTOI(vector<double> user_form)
{
	//Retreiving values
	x = user_form[0];
	y = user_form[1];
	z = user_form[2];
	theta = user_form[3] * (PI / 180);

	//Assign values
	//first row
	
	internal_form[0] = cos(theta);
	internal_form[1] = -sin(theta);
	internal_form[2] = 0;
	internal_form[3] = x;

	//second row
	internal_form[4] = sin(theta);
	internal_form[5] = cos(theta);
	internal_form[6] = 0;
	internal_form[7] = y;

	//third row
	internal_form[8] = 0;
	internal_form[9] = 0;
	internal_form[10] = 1;
	internal_form[11] = z;

	//fourth
	internal_form[12] = 0;
	internal_form[13] = 0;
	internal_form[14] = 0;
	internal_form[15] = 1;

	return internal_form;
}

/* Function: ITOU
* Takes the Transformation matrix and outputs the values of x, y,z and theta
*/
vector<double> mat_kin::ITOU(vector<double> internal_form)
{
	for (int i = 0; i < 3; i++) {
		user_form.push_back(internal_form[i * 1 + 3]);
	}
	user_form.push_back(acos(internal_form[0])*(180 / PI));

	return user_form;
}

/* Function: TMULT
* Takes two transformation matrices of 4x4 and multiplies them together
* Then ouputs the result as a matrix
*/
vector<double> mat_kin::TMULT(vector<double> t_matrix1, vector<double> t_matrix2)
{
	//Mutliply the Rotation Matirices together
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < WIDTH; col++) {
			// Multiply the row of A by the column of B to get the row, column of product.
			for (int inner = 0; inner < WIDTH; inner++) {
				r_matrix[row+col] += (t_matrix1[row+inner] * t_matrix2[inner+col]);
			}
		}
	}

	return r_matrix;
}

vector<double> mat_kin::TINVERT(vector<double> internal_form)
{
	double temp;
	temp = internal_form[0+1];
	internal_form[0+1] = internal_form[3+1];
	internal_form[3+1] = temp;

	temp = internal_form[0+2];
	internal_form[0+2] = internal_form[7+1];
	internal_form[7+1] = temp;

	temp = internal_form[3+2];
	internal_form[3+2] = internal_form[7+2];
	temp = internal_form[7+2];

	//Mutliply the Rotation Matirices and the Orign Vector
	for (int row = 0; row < 3; row++) {
		// Multiply the row of A by the column of B to get the row, column of product.
		for (int inner = 0; inner < 3; inner++) {
			point[row] += internal_form[row+inner] * internal_form[inner+3];

		}
		//cout << point[row] << ""<<endl;
	}
	//cout << ""<<endl;
	for (int row = 0; row < 3; row++) {
		internal_form[row+3] = -1 * point[row];
	}

	return internal_form;
}

vector<double> mat_kin::WHERE(vector<double> joint)
{

	the1 = joint[0] * (PI / 180);
	the2 = joint[1] * (PI / 180);
	d3 = joint[2];
	the4 = joint[3] * (PI / 180);


	internal_form[0+0] = cos(the1 + the2 - the4);
	internal_form[0+1] = sin(the1 + the2 - the4);
	internal_form[0+2] = 0;
	internal_form[0+3] = 195 * cos(the1) + 142 * cos(the1 + the2);

	internal_form[4*1+0] = sin(the1 + the2 - the4);
	internal_form[4*1+1] = -cos(the1 + the2 - the4);
	internal_form[4*1+2] = 0;
	internal_form[4*1+3] = 195 * sin(the1) + 142 * sin(the1 + the2);

	internal_form[4*2+0] = 0;
	internal_form[2*4+1] = 0;
	internal_form[2*4+2] = -1;
	internal_form[2*4+3] = -d3-75;//- d3 + 125;	//frame moved to base see demo1.m file (ask Caelan)s

	internal_form[3*4+0] = 0;
	internal_form[3*4+1] = 0;
	internal_form[3*4+2] = 0;
	internal_form[3*4+3] = 1;

	return internal_form;
}

vector<double> mat_kin::SOLVE(vector<double> oldJoints, vector<double> Tmatrix)
{
	vector<double> joints;
	double px = Tmatrix[0+3];
	double py = Tmatrix[1*4+3];
	double pz = Tmatrix[2*4+3];
	double r10 = Tmatrix[1*4+0];
	double r00 = Tmatrix[0+0];

	double joint1[2];
	double joint2[2];
	double joint3;
	double joint4[2];

	double A, r, gamma;

	joints[4*3+3] = 999;

	//theta2 = findTheta2(px,py, theta1[0], theta1[1]);	
	A = (px*px + py*py - 142 * 142 - 195 * 195) / (2 * 195 * 142);
	if (A*A > 1) {
		cout << "no solution" << endl;//do we really need this? it'll get checked in error checking
		joints[4 * 3 + 3] = 0;
	}
	joint2[0] = atan2(sqrt(1 - A*A), A);
	joint2[1] = atan2(-sqrt(1 - A*A), A);

	//theta1 = findTheta1(px, py, oldJoints[1]);
	for (int i = 0; i < 2; i++) {
		
		r = sqrt(pow((195 + 142 * cos(joint2[i])), 2) + pow(142 * sin(joint2[i]), 2));// jason
		if (r == 0) {
			cout << "no solution" << endl;//do we really need this? it'll get checked in error checking
			joints[4 * 3 + 3] = 0;
		}
		else {
			gamma = atan2(142* sin(joint2[i]), (195 +142* cos(joint2[i])));
			joint1[i] = atan2(py, px) - gamma;
		}	
	}

	joint3 = -pz - 75;			

	joint4[0] = joint1[0] + joint2[0] - atan2(r10, r00);
	joint4[1] = joint1[1] + joint2[1] - atan2(r10, r00);
	joint1[0] = joint1[0] * 180 / PI;
	joint1[1] = joint1[1] * 180 / PI;
	joint2[0] = joint2[0] * 180 / PI; 
	joint2[1] = joint2[1] * 180 / PI;
	joint4[0] = joint4[0] * 180 / PI;
	joint4[1] = joint4[1] * 180 / PI;

	//correcting joint 1 solution 1
	if (joint1[0] > 180)
	{
		joint1[0] = joint1[0] - 360;
	}
	else if (joint1[0] < -180)
	{
		joint1[0] = joint1[0] + 360;
	}
	//correcting joint 1 solution 2
	if (joint1[1] > 180)
	{
		joint1[1] = joint1[1] - 360;
	}
	else if (joint1[1] < -180)
	{
		joint1[1] = joint1[1] + 360;
	}
	
	//correcting joint 2 solution 1
	if (joint2[0] > 180)
	{
		joint2[0] = joint2[0] - 360;
	}
	else if (joint2[0] < -180)
	{
		joint2[0] = joint2[0] + 360;
	}
	//correcting joint 2 solution 2
	if (joint2[1] > 180)
	{
		joint2[1] = joint2[1] - 360;
	}
	else if (joint2[1] < -180)
	{
		joint2[1] = joint2[1] + 360;
	}

	//correcting joint 4 solution 1
	if (joint4[0] > 180)
	{
		joint4[0] = joint4[0] - 360;
	}
	else if (joint4[0] < -180)
	{
		joint4[0] = joint4[0] + 360;
	}
	//correcting joint 4 solution 2
	if (joint4[1] > 180)
	{
		joint4[1] = joint4[1] - 360;
	}
	else if (joint4[1] < -180)
	{
		joint4[1] = joint4[1] + 360;
	}

	//place results into output array
	joints[0+0] = joint1[0];
	joints[4*1+0] = joint1[1];
	joints[0+1] = joint2[0];
	joints[4*1+1] = joint2[1];
	joints[0+2] = joint3;
	joints[1*4+2] = joint3;
	joints[0+3] = joint4[0];
	joints[1*4+3] = joint4[1];

	//** Error Checking **

	for (int i = 0; i < 4; i++) {
		errJoint1[i] = joints[i];
	}

	for (int i = 0; i < 4; i++) {
		errJoint2[i] = joints[4+i];
	}

	if (errorFound(errJoint1, 1) && errorFound(errJoint2, 1)) {
		joints[4*3+3] = 0;
	}
	else if (errorFound(errJoint1, 1)) {
		joints[4*3+3] = 1;
	}
	else if (errorFound(errJoint2, 1)) {
		joints[4*3+3] = 2;
	}
	return joints;
}

void mat_kin::printMatrix(vector<double> matrix, int height, int width) {

	for (int y = 0; y< height; y++) {
		for (int x = 0; x< width; x++) {
			cout << setw(15) << matrix[y+x];
		}
		cout << endl;
	}
}

bool mat_kin::errorFound(vector<double> values, int selection) {

	if (selection == 1){		// Check Joint values
		if ((round(values[0]) > 150 || (round(values[0]))< -150))
		{
			cout << "ERROR: Joint 1 limit" << endl;
			return true;
		}
		else if ((round(values[1]) > 100 || (round(values[1]) < -100))) {
			cout << "ERROR: Joint 2 limit" << endl;
			return true;
		}
		else if ((values[2]) < -200 || ((values[2]) > -100)) {
			cout << "ERROR: Joint 3 limit" << endl;
			return true;
		}
		else if ((round(values[3]) > 160 || (round(values[3]) < -160))) {
			cout << "ERROR: Joint 4 limit" << endl;
			return true;
		}
		else {
			return false;
		}
	}
	else if (selection == 2) {		// Check Joint Velocities

		if (round(values[0]) > 150 || round(values[0]) < -150) {
			cout << "ERROR: Velocity Joint 1 limit" << endl;
			return true; 
		}
		else if (round(values[1]) > 150 || round(values[1]) < -150) {
			cout << "ERROR: Velocity Joint 2 limit" << endl;
			return true;
		}
		else if ((values[2]) > 50 || values[2] < -50) {
			cout << "ERROR: Velocity Joint 3 limit" << endl;
			return true;
		}
		else if (round(values[3]) >150 || round(values[3]) < -150) {
			cout << "ERROR: Velocity Joint 4 limit" << endl;
			return true;
		}
		else {
			return false;
		}
	}
	else if (selection == 3) {		// Check Joint Acceleration
		if (round(values[0]) > 600 || round(values[0]) < -600) {
			cout << "ERROR: Acceleration Joint 1 limit" << endl;
			return true;
		}
		else if (round(values[1]) > 600 || round(values[1]) < -600) {
			cout << "ERROR: Acceleration Joint 2 limit" << endl;
			return true;
		}
		else if (round(values[2]) <= -200 || round(values[2])> 200) {
			cout << "ERROR: Acceleration Joint 3 limit" << endl;
			return true;
		}
		else if (round(values[3]) >600 || round(values[3]) < -600) {
			cout << "ERROR: Accelerationy Joint 4 limit" << endl;
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}
