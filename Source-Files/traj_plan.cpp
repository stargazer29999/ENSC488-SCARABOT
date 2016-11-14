/* Traj_plan.cpp
*This member class includes all the functions required to 
*caluate and print a direcrete spline for a give set of frames
*/

#include "traj_plan.h"

using namespace std;


/* DiscreteTrajectory function:
* Input: the viaPoints as Joint values from the User interface
* Calls other functions to change values 
* Ouput: the discrete values of: joint value, joint veloctiy and joint acceleration
*/
double*** traj_plan::discreteTrajectory(double** viaPoints, int numVia) {
	double joint[5];

	//double Output[4][TIME][WIDTH];	//TIME is the final time divided by the time resolution

	Output = new double**[4];		//RENAME AT A LATER POINT
	for (int i = 0; i < 4; i++) {
		Output[i] = new double*[TIME];
		for (int j = 0; j < TIME; j++) {
			Output[i][j] =new double[WIDTH];
		}
	}

	//Solve for a trajectory function of each joint with the given parameters
	for (int i = 0; i < 4; i++) { //each joint value is in the columns of viaPoints
		for (int j = 0; j < 5; j++) {
			joint[j] = viaPoints[j][i];
		}
		Output[i]= calcSpline(joint, numVia); 
	}

	//Once everything is calculated and before sending the matrix print the results to a file
	printTrajectory("joint1.txt", Output[0]);
	printTrajectory("joint2.txt", Output[1]);
	printTrajectory("joint3.txt", Output[2]);
	printTrajectory("joint4.txt", Output[3]);

	return Output;
}

/* printTrajectory function:
* Input:the joint file name and the values of the time, joint angle, velocity, acceleration as an array
* With the given input this function should output a text file with the following format per line" time	   joint_value    velocity    acceleration" 
* Return Ouput: None
*/
void traj_plan::printTrajectory(string filename,  double** jointValues) {

	ofstream myfile(filename);
	if (myfile.is_open())
	{
		for (int i = 0; i < TIME; i++) {
			for (int j = 0; j < WIDTH; j++) {
				myfile << jointValues[i][j]<<"\t";
			}
			myfile << endl;
		}
		myfile.close();
	}
	else {
		cout << "Unable to open file";
	}

}

/* calcSpline function:
* Input: pointer to a given joint
* Calculates the spline for a given joint
* Ouput: ____
*/
double** traj_plan::calcSpline(double* joint, int numVia) {
	
	spline= new double*[TIME];
	for (int j = 0; j < TIME; j++) {
		spline[j] = new double[WIDTH];
	}

	switch (numVia) {
	case 0: //there is only the START and GOAL vias


		break;
	case 1:  //Along with the START and GOAL vias there is ONE intermediate point
		/* //V1 for 2 ptn case (start and goal)
		v1 = 6 * (delta0 / (4 * h0) + h0*delta1 / (4 * h0*h0) + h1*delta0 / (4 * h0*h0) + delta1 / (4 * h0*h0));
		*/

		coeff = calcCoeff(yi, yi_1, v1, vi_1, hi); // what to do with this? (yi, yi_1 are part of the 'joint' array
		spline = calcDiscreteSpline(0, FINAL, RES, coeff);


		break;
	case 2:	//There are TWO intermediate points



		break;
	case 3: //There are THREE intermediate points



		break;	
	default: //This should never happend
		break;
	}

	return spline;
}

/* calcCoeff function:
* Input: the joint values of two points the velocity at two points and the time difference (hi=ti_1-ti), where ti is real time (i.e tow)
* Calculates the coefficents ai, bi, ci and di for a given vi value
* Ouput: Coefficent values as 4x1 array;
*/
double* traj_plan::calcCoeff(double yi, double yi_1, double vi, double vi_1, double hi) {
	 double coeff[4]; //ai bi ci di
	 
	 coeff[0] = yi;
	 coeff[1] = vi*hi;
	 coeff[2] = 3 * (yi_1 - yi) - 2 * vi - vi_1;
	 coeff[3] = -2 * (yi_1 - yi) + vi*hi + vi_1*hi;

	return coeff;
}


/* calcDiscreteSpline function:
* Input: the starting time of the spline (t0),the ending time of the spline (tf), the time resolution (timeRes), the coefficent values
* Calculates the discrete values of joint, velocity and acceleration for the given time period
* Ouput: along with the calculated values it outputs the time;
*/

double** traj_plan::calcDiscreteSpline(double t0, double tf, double timeRes, double* coeff) {		//KARA change Time Resolution to a consatant value
	int t = t0;
	int i = 0;

	discreteSpline=new double*[(int)((tf - t0) / timeRes)];
	for (int i = 0; i < (int)((tf - t0) / timeRes); i++) {
		discreteSpline[i] = new double[WIDTH];
	}

	while (t <= tf) {				//Kara:is the last entry suposed to be at the final time or the one before?
		discreteSpline[0][i] = t;
		discreteSpline[1][i] = coeff[0] + coeff[1] * t + coeff[2] * t*t + coeff[3] * t*t*t;
		discreteSpline[2][i] = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t*t;
		discreteSpline[3][i] = 2 * coeff[2] + 6 * coeff[3] * t;

		t + timeRes;
		i++;
	}

	return discreteSpline;

}
