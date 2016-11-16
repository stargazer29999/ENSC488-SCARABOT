/* Traj_plan.cpp
*This member class includes all the functions required to 
*caluate and print a direcrete spline for a give set of frames
*/

#include "traj_plan.h"

using namespace std;
traj_plan::traj_plan() {
	
	joint = new double[5];
	time = new double[5];
	
	Output = new double**[4];		//RENAME AT A LATER POINT
	for (int i = 0; i < 4; i++) {
		Output[i] = new double*[4 * RES];
		for (int j = 0; j < 4 * RES; j++) {
			Output[i][j] = new double[WIDTH];
		}
	}

	tau = new double[5];

	coeff = new double[4];

	coefficient = new double[4]; //ai bi ci di

	discreteSpline = new double*[RES];
	for (int j = 0; j < RES; j++) {
		discreteSpline[j] = new double[WIDTH]; //Unhandled exception at 0x75EE5B68 in RRPR.exe: Microsoft C++ exception: std::bad_alloc at memory location 0x0073EE20.
	}

}

/* DiscreteTrajectory function:
* Input: the viaPoints as Joint values from the User interface
* Calls other functions to change values 
* Ouput: the discrete values of: joint value, joint veloctiy and joint acceleration
*/
double*** traj_plan::discreteTrajectory(double** viaPoints, int numVia) {
	


	cout << endl << "***************** Inside Trajectory Planner *************************" << endl;
	//Solve for a trajectory function of each joint with the given parameters
	for (int i = 0; i < 4; i++) { //each joint value is in the columns of viaPoints
		for (int j = 0; j < 5; j++) {
			joint[j] = viaPoints[j][i];
				time[j] = viaPoints[j][4];
		}

		//Comment out the follwoing section once the code works to save space
		cout << "joint "<< (i+1) <<" values: " << endl;
		//cout << setw(15) << "j1" << setw(15) << "j2" << setw(15) << "j3" << setw(15) << "j4" << setw(15) << "time (ms)" << endl;
			for (int y = 0; y< 5; y++) {
					cout << setw(15) << joint[y];// << "\t";
				}
				cout << endl;
			
				cout << "Time (ms): ";
			for (int y = 0; y< 5; y++) {
				cout << setw(15) << time[y];// << "\t";
			}
			cout << endl;
			//end of comment out section

		Output[i]= calcSpline(joint, numVia, i,time); 

		cout << "Used in Debugging";
		if (i = 0) {
			for (int ii = 0; ii < 2 * RES; ii++) {
				for (int jj = 0; jj < 5; jj++) {
					cout << setw(10) << Output[0][ii][jj];
				}
				cout << endl;
			}
		}


}
	switch (numVia) {
	case 0:
		steps = RES;
		break;
	case 1:
		steps = 2 * RES;
		break;
	case 2:
		steps = 3 * RES;
		break;
	case 3:
		steps = 4 * RES;
		break;
	default:
		break;
}

	//Once everything is calculated and before sending the matrix print the results to a file
	printTrajectory("joint1.txt", Output[0], steps);
	printTrajectory("joint2.txt", Output[1], steps);
	printTrajectory("joint3.txt", Output[2], steps);
	printTrajectory("joint4.txt", Output[3], steps);

	return Output;
}

/* printTrajectory function:
* Input:the joint file name and the values of the time, joint angle, velocity, acceleration as an array
* With the given input this function should output a text file with the following format per line" time	   joint_value    velocity    acceleration" 
* Return Ouput: None
*/
void traj_plan::printTrajectory(string filename,  double** jointValues, int steps) {
	cout << endl << "** Inside printTrajectory ******" << endl;

	ofstream myfile(filename);
	if (myfile.is_open())
	{
		for (int i = 0; i < steps; i++) {
			for (int j = 0; j < WIDTH; j++) {
				myfile << jointValues[i][j]<<"\t";
			}
			myfile << endl;
		}
		myfile.close();
	}
	

}

/* calcSpline function:
* Input: pointer to a given joint, We also need to add some time (Tau) to calculate h
* Calculates the spline for a given joint
* Ouput: ____
*/
double** traj_plan::calcSpline(double* joint, int numVia, int i, double *time /*, double* tau */){
	cout << endl << "** Inside calcSpline ******" << endl;
	
	spline = new double*[RES*numVia + RES];
	for (int j = 0; j < (RES*numVia + RES); j++) {
		spline[j] = new double[WIDTH + 1];
	}

	//intialize the tau used by all some produce invalud results but then are not used for calulations
	//Kara; check this over
//	tau[0] = time[0];
//	tau[1] = tau[0] + time[1];

	switch (numVia) {
	case 0: //there is only the START and GOAL vias
		//Use Parabolic Blending
		/*
		
		if (i=1||i=2||i=4){
		maxAcc=revolMaxAcc

		}
		else{
		maxAcc=prisMaxAcc
		}

		tn=(to+tf)/2;
		yn=(joint[0]+joint[4])/2;
		if(tf*tf<4*(joint[4]-joint[0])/maxAcc){
		cout<<"NO Solution"<<endl;
		}
		else{

		tb=tf/2-sqrt((pow(maxAcc,2)*tf*tf-4*maxAcc*(joint[4]-joint[0])/(2*maxAcc);
		
		vb=
		
		}

		*/
		break;
	case 1:  //Along with the START and GOAL vias there is ONE intermediate point
		vi_1 = 0; //since the GOAL frame has zero velocity
	
		 //V1 for 2 ptn case (start and goal)
		delta0 = joint[1] - joint[0];
		delta1 = joint[4] - joint[1];

		
		//tau[2] = tau[1] + time[4]; //Kara: check this over

		//h_0 = tau[1] - tau[0];
		//h_1 = tau[2] - tau[1];
		h_0 = time[1]; //time to travel from start to intermediate frame, specified by user
		h_1 = time[2];	//time  to travel from intermediate frame to goal, specified by user

		v1 =( 3*delta1*h_0*h_0 + 3*delta0*h_1*h_1)/(2*h_0*h_0*h_1 + 2*h_0*h_1*h_1); //determined through MATLAB
		

		//This method will only work for when we have two points

		//Calcuate the discrete values 
		coeff = calcCoeff(joint[0], joint[1], 0, v1, h_0);
		temp = calcDiscreteSpline(0, time[1],  coeff);
		for (int i = 0; i < RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i][j];
			}
		}

		coeff= calcCoeff(joint[1], joint[2], v1, v2, h_1);
		temp = calcDiscreteSpline(time[1], time[1]+time[4],coeff);

		//KARA:the result of i=1 needs to be placed on spline after the results of i=0, and so on and so forth
		for (int i = RES; i < 2*RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - RES][j];
			}
		}

		break;
	case 2:	//There are TWO intermediate points
		
		delta0 = joint[1] - joint[0];
		delta1 = joint[2] - joint[1];
		delta2 = joint[4] - joint[2];

		//Kara; check this over
		//tau[2] = tau[1] + time[2];
		//tau[3] = tau[2] + time[4];
		
		h_0 = time[1];// tau[1] - tau[0];
		h_1 = time[2];// tau[2] - tau[1];
		h_2 = time[4];// tau[3] - tau[2];
		
		v1 = (3*(-delta2*h_0*h_0*h_1*h_1 + 2*delta1*h_0*h_0*h_1*h_2 + delta1*h_0*h_0*h_2*h_2 + 2*delta0*pow(h_1,3)*h_2 + 2*delta0*h_1*h_1*h_2*h_2))/
			(h_0*h_1*h_2*(4*h_0*h_1 + 3*h_0*h_2 + 4*h_1*h_2 + 4*h_1*h_1));
		
		v2 = (3*(2*delta2*h_0*h_0*h_1*h_1 + delta1*h_0*h_0*h_2*h_2 + 2*delta2*h_0*pow(h_1,3) + 2*delta1*h_0*h_1*h_2*h_2 - delta0*h_1*h_1*h_2*h_2))/
			(h_0*h_1*h_2*(4*h_0*h_1 + 3*h_0*h_2 + 4*h_1*h_2 + 4*h_1*h_1));
		
	
		//KARA:the result of i=1 needs to be placed on spline after the results of i=0, and so on and so forth
		//Kara: Currently the spline is just being overwritten every time

		coeff = calcCoeff(joint[0], joint[1], 0, v1, h_0);
		spline = calcDiscreteSpline(0, time[1], coeff);
		t[0] = time[0] + time[1];

		coeff = calcCoeff(joint[1], joint[2], v1, v2, h_1);
		temp = calcDiscreteSpline(time[1], t[0], coeff);
		for (int i = RES; i < 2 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - RES][j];
			}
		}
		coeff = calcCoeff(joint[2], joint[3], v2, v3, h_2);
		temp = calcDiscreteSpline(t[0], t[0]+time[4], coeff);
		for (int i = 2*RES; i < 2 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - 2*RES][j];
			}
		}

		break;
	case 3: //There are THREE intermediate points

		delta0 = joint[1] - joint[0];
		delta1 = joint[2] - joint[1];
		delta2 = joint[3] - joint[2];
		delta3 = joint[4] - joint[3];
		
		//Kara; check this over
		//tau[2] = tau[1] + time[2];
		//tau[3] = tau[2] + time[3];
		//tau[4] = tau[3] + time[4];

		h_0 = time[1];// tau[1] - tau[0];
		h_1 = time[2];//tau[2] - tau[1];
		h_2 = time[3];//tau[3] - tau[2];
		h_3 = time[4];//tau[4] - tau[3];
		
		v1 = (3*(4*delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) + 2*delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - 2*delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) + delta3*h_0*h_0*h_1*h_1*h_2*h_2 + delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) + 4*delta0*h_1*h_1*pow(h_2, 3)*h_3 + 3*delta0*pow(h_1,3)*h_2*pow(h_3, 2) + 4*delta0*pow(h_1,3)*h_2*h_2*h_3 + 2*delta1*h_0*h_0*pow(h_2, 3)*h_3 + 3*delta1*h_0*h_0*h_1*h_2*pow(h_3, 2) + 4*delta1*h_0*h_0*h_1*h_2*h_2*h_3 - 2*delta2*h_0*h_0*h_1*h_1*h_2*h_3))/
			(2*h_0*h_1*h_2*h_3*(3*h_0*h_2*h_2 + 4*h_1*h_2*h_2 + 4*h_1*h_1*h_2 + 3*h_1*h_1*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		
		v2 = (3*(delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) + 2*delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) - delta3*h_0*h_0*h_1*h_1*h_2*h_2 - delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) - delta0*h_1*h_1*pow(h_2, 3)*h_3 + delta1*h_0*h_0*pow(h_2, 3)*h_3 + 2*delta2*h_0*pow(h_1,3)*pow(h_3, 2) - delta3*h_0*pow(h_1,3)*h_2*h_2 - delta3*h_0*pow(h_1,3)*pow(h_3, 2) + 2*delta1*h_0*h_1*h_2*h_2*pow(h_3, 2) + 2*delta2*h_0*h_0*h_1*h_1*h_2*h_3 + 2*delta1*h_0*h_1*pow(h_2, 3)*h_3 + 2*delta2*h_0*pow(h_1,3)*h_2*h_3))/
			(h_0*h_1*h_2*h_3*(3*h_0*h_2*h_2 + 4*h_1*h_2*h_2 + 4*h_1*h_1*h_2 + 3*h_1*h_1*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		
		v3 = (3*(delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) - delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - 2*delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) + 4*delta3*h_0*h_0*h_1*h_1*h_2*h_2 + 4*delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) - 2*delta2*h_0*pow(h_1,3)*pow(h_3, 2) + 4*delta3*h_0*h_1*h_1*pow(h_2, 3) + 4*delta3*h_0*pow(h_1,3)*h_2*h_2 + 3*delta3*h_0*h_0*h_1*pow(h_2, 3) + 4*delta3*h_0*pow(h_1,3)*pow(h_3, 2) - 2*delta1*h_0*h_1*h_2*h_2*pow(h_3, 2) + 4*delta3*h_0*h_1*h_1*h_2*pow(h_3, 2) + 3*delta3*h_0*h_0*h_1*h_2*pow(h_3, 2)))/
			(2*h_0*h_1*h_2*h_3*(3*h_0*h_2*h_2 + 4*h_1*h_2*h_2 + 4*h_1*h_1*h_2 + 3*h_1*h_1*h_3 + 4*h_0*h_1*h_2 + 3*h_0*h_1*h_3 + 3*h_0*h_2*h_3 + 4*h_1*h_2*h_3));
		

		//KARA:the result of coeff[1] needs to be placed on spline after the results of coeff[0], and so on and so forth
		//Kara: Is there a more elgant way to doe this?
		coeff= calcCoeff(joint[0], joint[1], 0, v1, h_0);
		spline = calcDiscreteSpline(0, time[0], coeff);


		coeff = calcCoeff(joint[1], joint[2], v1, v2, h_1);
		temp = calcDiscreteSpline(time[0], t[0], coeff);
		for (int i = RES; i < 2 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - RES][j];
			}
		}
		t[1] = t[0] + time[2];

		coeff = calcCoeff(joint[2], joint[3], v2, v3, h_2);
		temp = calcDiscreteSpline(t[0], t[1], coeff);
		for (int i = 2 * RES; i < 2 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - 2 * RES][j];
			}
		}
		t[2] = t[1] + time[3];

		coeff= calcCoeff(joint[3], joint[4], v3, v4, h_3);
		temp = calcDiscreteSpline(t[1],t[2], coeff);
		for (int i = 3 * RES; i < 4 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - 3 * RES][j];
			}
		}

		coeff = calcCoeff(joint[4], joint[5], v4, 0, h_4);
		temp= calcDiscreteSpline(t[2], t[2] +time[4], coeff);
		for (int i = 4 * RES; i < 5 * RES; i++) {
			for (int j = 0; j < (WIDTH + 1); j++) {
				spline[i][j] = temp[i - 4 * RES][j];
			}
		}

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
	cout << endl << "** Inside calcCoeff ******" << endl;

	 coefficient[0] = yi;
	 coefficient[1] = vi*hi;
	 coefficient[2] = 3 * (yi_1 - yi) - 2 * vi - vi_1;
	 coefficient[3] = -2 * (yi_1 - yi) + vi*hi + vi_1*hi;

	return coefficient;
}


/* calcDiscreteSpline function:
* Input: the starting time of the spline (t0),the ending time of the spline (tf), the time resolution (timeRes), the coefficent values
* Calculates the discrete values of joint, velocity and acceleration for the given time period
* Ouput: along with the calculated values it outputs the time;
*/

double** traj_plan::calcDiscreteSpline(double t0, double tf,  double* coeff) {		//KARA change Time Resolution to a consatant value
	cout << endl << "** Inside calcDiscreteSpline ******" << endl;
	int t = t0;
	int i = 0;

	deltaT = (tf - t0) / RES;


	while (t <= tf) {				//Kara:is the last entry suposed to be at the final time or the one before?
		discreteSpline[0][i] = t;
		discreteSpline[1][i] = fmod(coeff[0] + coeff[1] * t + coeff[2] * t*t + coeff[3] * t*t*t,360); //modulo 360 degree arithmatic to ensure angle is valid
		if (discreteSpline[1][i] > 180) {		//changing values to the range [-180 180]
			discreteSpline[1][i] = 180 - discreteSpline[1][i];
		}
		discreteSpline[2][i] = fmod(coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t*t, 360); //Kara: do we need modulo 360 degree arithmatic for velocity?
		discreteSpline[3][i] = 2 * coeff[2] + 6 * coeff[3] * t;

		t=t + deltaT;
		i++;
	}

	return discreteSpline;

}

void traj_plan::printM(double** matrix, int height, int width) {

	for (int y = 0; y< height; y++) {
		for (int x = 0; x< width; x++) {
			cout << setw(15) << matrix[y][x];// << "\t";
		}
		cout << endl;
	}
	//cout <<endl;
}
