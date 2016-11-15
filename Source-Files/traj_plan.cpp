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
* Input: pointer to a given joint, We also need to add some time (Tau) to calculate h
* Calculates the spline for a given joint
* Ouput: ____
*/
double** traj_plan::calcSpline(double* joint, int numVia /* double* tau */){
	
	spline= new double*[TIME];
	for (int j = 0; j < TIME; j++) {
		spline[j] = new double[WIDTH];
	}

	switch (numVia) {
	case 0: //there is only the START and GOAL vias


		break;
	case 1:  //Along with the START and GOAL vias there is ONE intermediate point
		vi_1 = 0; //since the GOAL frame has zero velocity
		double h_0, h_1;
		double delta0, delta1;	
		
		/* //V1 for 2 ptn case (start and goal)
		delta0 = joint[1] - joint[0];
		delta1 = joint[2] - joint[1];
		
		h_0 = tau[1] - tai[0];
		h_1 = tau[2] - tau[1];
		
		v1 = (3*delta1*h_0^2 + 3*delta0*h_1^2)/(2*h_0^2*h_1 + 2*h_0*h_1^2); //determined through MATLAB
		*/
		//This method will only work for when we have two points
		/*
		coeff[0] = calcCoeff(joint[0],joint[1],0,v1,h_0);
		coeff[1] = calcCoeff(joint[1],joint[2],v1,v2,h_1);
		*/
		coeff = calcCoeff(yi, yi_1, v1, vi_1, hi); // what to do with this? (yi, yi_1 are part of the 'joint' array) (Caelan: I'm not 100% sure, most likely we need to sample it or display in) 
		spline = calcDiscreteSpline(0, FINAL, RES, coeff);


		break;
	case 2:	//There are TWO intermediate points
		double h_0, h_1, h_2;
		double delta0, delta1, delta2;
			
		/* 
		delta0 = joint[1] - joint[0];
		delta1 = joint[2] - joint[1];
		delta2 = joint[3] - joint[2];
		
		h_0 = tau[1] - tau[0];
		h_1 = tau[2] - tau[1];
		h_2 = tau[3] - tau[2];
		
		v1 = (3*(- delta2*h_0^2*h_1^2 + 2*delta1*h_0^2*h_1*h_2 + delta1*h_0^2*h_2^2 + 2*delta0*h_1^3*h_2 + 2*delta0*h_1^2*h_2^2))/(h_0*h_1*h_2*(4*h_0*h_1 + 3*h_0*h_2 + 4*h_1*h_2 + 4*h_1^2));
		
		v2 = (3*(2*delta2*h_0^2*h_1^2 + delta1*h_0^2*h_2^2 + 2*delta2*h_0*h_1^3 + 2*delta1*h_0*h_1*h_2^2 - delta0*h_1^2*h_2^2))/(h_0*h_1*h_2*(4*h_0*h_1 + 3*h_0*h_2 + 4*h_1*h_2 + 4*h_1^2));
		
	
		//Here we will call the coefficient calculator function for m cubic segment (i.e. 4 times in this case)
		//I think we should store the coeffecients as an array in a array (is this wise?)
		
		coeff[0] = calcCoeff(joint[0],joint[1],0,v1,h_0);
		coeff[1] = calcCoeff(joint[1],joint[2],v1,v2,h_1);
		coeff[2] = calcCoeff(joint[2],joint[3],v2,v3,h_2);
		*/
		break;
	case 3: //There are THREE intermediate points
		double h_0, h_1, h_3;
		double h_4; //time btw. last via and goal?
		double delta0, delta1, delta2, delta3;
		
		/*
		delta0 = joint[1] - joint[0];
		delta1 = joint[2] - joint[1];
		delta2 = joint[3] - joint[2];
		delta3 = joint[4] - joint [3];
		
		h_0 = tau[1] - tai[0];
		h_1 = tau[2] - tau[1];
		h_2 = tau[3] - tau[2];
		h_3 = tau[4] - tau[3];
		
		v1 = (3*(4*delta0*h_1^2*h_2^2*h_3^2 + 2*delta1*h_0^2*h_2^2*h_3^2 - 2*delta2*h_0^2*h_1^2*h_3^2 + delta3*h_0^2*h_1^2*h_2^2 + delta3*h_0^2*h_1^2*h_3^2 + 4*delta0*h_1^2*h_2^3*h_3 + 3*delta0*h_1^3*h_2*h_3^2 + 4*delta0*h_1^3*h_2^2*h_3 + 2*delta1*h_0^2*h_2^3*h_3 + 3*delta1*h_0^2*h_1*h_2*h_3^2 + 4*delta1*h_0^2*h_1*h_2^2*h_3 - 2*delta2*h_0^2*h_1^2*h_2*h_3))/(2*h_0*h_1*h_2*h_3*(3*h_0*h_2^2 + 4*h_1*h_2^2 + 4*h_1^2*h_2 + 3*h_1^2*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		
		v2 = (3*(delta1*h_0^2*h_2^2*h_3^2 - delta0*h_1^2*h_2^2*h_3^2 + 2*delta2*h_0^2*h_1^2*h_3^2 - delta3*h_0^2*h_1^2*h_2^2 - delta3*h_0^2*h_1^2*h_3^2 - delta0*h_1^2*h_2^3*h_3 + delta1*h_0^2*h_2^3*h_3 + 2*delta2*h_0*h_1^3*h_3^2 - delta3*h_0*h_1^3*h_2^2 - delta3*h_0*h_1^3*h_3^2 + 2*delta1*h_0*h_1*h_2^2*h_3^2 + 2*delta2*h_0^2*h_1^2*h_2*h_3 + 2*delta1*h_0*h_1*h_2^3*h_3 + 2*delta2*h_0*h_1^3*h_2*h_3))/(h_0*h_1*h_2*h_3*(3*h_0*h_2^2 + 4*h_1*h_2^2 + 4*h_1^2*h_2 + 3*h_1^2*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		
		v3 = (3*(delta0*h_1^2*h_2^2*h_3^2 - delta1*h_0^2*h_2^2*h_3^2 - 2*delta2*h_0^2*h_1^2*h_3^2 + 4*delta3*h_0^2*h_1^2*h_2^2 + 4*delta3*h_0^2*h_1^2*h_3^2 - 2*delta2*h_0*h_1^3*h_3^2 + 4*delta3*h_0*h_1^2*h_2^3 + 4*delta3*h_0*h_1^3*h_2^2 + 3*delta3*h_0^2*h_1*h_2^3 + 4*delta3*h_0*h_1^3*h_3^2 - 2*delta1*h_0*h_1*h_2^2*h_3^2 + 4*delta3*h_0*h_1^2*h_2*h_3^2 + 3*delta3*h_0^2*h_1*h_2*h_3^2))/(2*h_0*h_1*h_2*h_3*(3*h_0*h_2^2 + 4*h_1*h_2^2 + 4*h_1^2*h_2 + 3*h_1^2*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		
		coeff[0] = calcCoeff(joint[0],joint[1],0,v1,h_0);
		coeff[1] = calcCoeff(joint[1],joint[2],v1,v2,h_1);
		coeff[2] = calcCoeff(joint[2],joint[3],v2,v3,h_2);
		coeff[3] = calcCoeff(joint[3],joint[4],v3,v4,h_3);
		coeff[4] = calcCoeff(joint[4],joint[5],v4,0,h_4); 
 		*/

		break;	
	default: //This should never happend
		break;
	}

	return spline;
}

/* calcCoeff function:
* Input: the joint values of two points the velocity at two points and the time difference (hi=ti_1-ti), where ti is real time (i.e tau)
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
