/* Traj_plan.cpp
*This member class includes all the functions required to
*caluate and print a direcrete spline for a give set of frames
*/
#include <chrono>
#include <thread>
#include "traj_plan.h"

using namespace std;
traj_plan::traj_plan() {
	
	if (remove("Joints.txt") != 0&& remove("numPoints.txt") != 0)
	{
		//error in deleting file;	
	}
	else
	{


	}
	joint = new double[5];
	time = new double[5];

	Output = new double**[4];		//RENAME AT A LATER POINT

	tau = new double[5]; //Is thie used anywhere?

	coeff = new double[4];

	coefficient = new double[4]; //ai bi ci di


	}

	/* DiscreteTrajectory function:
	* Input: the viaPoints as Joint values from the User interface
	* Calls other functions to change values
	* Ouput: the discrete values of: joint value, joint veloctiy and joint acceleration
	*/
	double*** traj_plan::discreteTrajectory(double** viaPoints, int numVia, int numPoints) {
		using namespace std::this_thread; // sleep_for, sleep_until
		using namespace std::chrono; // nanoseconds, system_clock, seconds
		for (int i = 0; i < 4; i++) {
			Output[i] = new double*[numPoints];
			for (int j = 0; j < numPoints; j++) {
				Output[i][j] = new double[WIDTH];
			}
		}

		appPrintTrajectory("numPoints.txt", numPoints);
		appPrintTrajectory("Joints.txt", -9999);

		cout << endl << "***************** Inside Trajectory Planner *************************" << endl;
		//Solve for a trajectory function of each joint with the given parameters
		for (int i = 0; i < 4; i++) { //each joint value is in the columns of viaPoints
			for (int j = 0; j < 5; j++) {
				joint[j] = viaPoints[j][i];
				time[j] = viaPoints[j][4];
			}

			cout << "joint " << (i + 1) << " values: " << endl;
			for (int y = 0; y < 5; y++) {
				cout << setw(10) << joint[y];
			}
			cout << endl;

			cout << "Time (ms): ";
			for (int y = 0; y < 5; y++) {
				cout << setw(5) << time[y];
			}
			cout << endl;


			Output[i] = calcSpline(joint, numVia, i, time, numPoints);

			//Modulo 360 arithmatic
			if (i == 0 || i == 1 || i == 3) {
				ii = 0;
				while ((ii <= (numPoints)) && ((int)Output[i][ii][0] != 999)) {
					Output[i][ii][1] = fmod(Output[i][ii][1], 360);
					if (Output[i][ii][1] > 180) {		//changing values to the range [-180 180]
						Output[i][ii][1] = Output[i][ii][1] - 360;
					}
					else if (Output[i][ii][1] < -180) {
						Output[i][ii][1] = Output[i][ii][1] + 360;
					}
					ii++;
				}
			}


			//cout << "Used for Debugging, joint " << (i + 1) << endl;
			//cout << setw(12) << "time" << setw(12) << "joint val" << setw(12) << "velocity" << setw(12) << "accel" << endl;
			ii = 0;
			while ((ii <= (numPoints)) && ((int)Output[i][ii][0] != 999)) {
				for (int jj = 0; jj < 4; jj++) {
					//cout << setw(12) << Output[i][ii][jj];
					appPrintTrajectory("Joints.txt", Output[i][ii][jj]);

					//				printResults(Output, steps, ii, jj);
					sleep_for(milliseconds(10));
				}
				//cout << endl;
				appPrintTrajectory("Joints.txt", -9999);
				ii++;
			}
			cout << endl;
		}
	


		/*	switch (numVia) {
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
		

		//Once everything is calculated and before sending the matrix print the results to a file
		printTrajectory("joint1.txt", Output[1], steps);
		printTrajectory("joint2.txt", Output[1], steps);
		printTrajectory("joint3.txt", Output[2], steps);
		printTrajectory("joint4.txt", Output[3], steps);
*/
		return Output;
	}

	/* printTrajectory function:
	* Input:the joint file name and the values of the time, joint angle, velocity, acceleration as an array
	* With the given input this function should output a text file with the following format per line" time	   joint_value    velocity    acceleration"
	* Return Ouput: None
	*/
	void traj_plan::printTrajectory(string filename, double** jointValues, int steps) {
		cout << endl << "** Inside printTrajectory ******" << endl;

		ofstream myfile(filename);
		if (myfile.is_open())
		{
			for (int i = 0; i < steps; i++) {
				for (int j = 0; j < WIDTH; j++) {
					myfile << jointValues[i][j] << "\t";
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
	double** traj_plan::calcSpline(double* joint, int numVia, int i, double *time, int numPoints) {

		spline = new double*[numPoints];
		for (int j = 0; j <= (numPoints); j++) {
			spline[j] = new double[WIDTH];
		}
		via0 = (int)ceil(time[1] / RES);
		via1 = (int)ceil(tau[0] / RES);
		via2 = (int)ceil(tau[1] / RES);
		via3 = (int)ceil(tau[2] / RES);
		tau[0] = time[0] + time[1];
		tau[1] = tau[0] + time[2];
		tau[2] = tau[1] + time[3];

		cout << endl << "** Inside calcSpline ******" << endl;

		//intialize the tau used by all some produce invalud results but then are not used for calulations

		switch (numVia) {
		case 0: //there is only the START and GOAL vias
				//Use a  quanitic ploynomial instead use solution on page 214 of test book

			coeff[0] = joint[0];
			coeff[1] = 0;
			coeff[2] = 3 * (joint[4] - joint[0]) / pow(time[4], 2);
			coeff[3] = -2 * (joint[4] - joint[0]) / (pow(time[4], 3));

			t = 0;
			ii = 0;

			deltaT = RES;

			while (t <= time[4]) {				//Kara: the last entry suposed to be at the final time
				spline[ii][0] = t;
				spline[ii][1] = coeff[0] + coeff[1] * t + coeff[2] * pow(t, 2) + coeff[3] * pow(t, 3); //modulo 360 degree arithmatic to ensure angle is valid
				spline[ii][2] = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t*t;
				spline[ii][3] = 2 * coeff[2] + 6 * coeff[3] * t;

				t = t + deltaT;
				ii++;
			}
			//Make a line of invalid values to represent the end of results
			/*		spline[RES + 1][0] = 999;
			spline[RES + 1][1] = 999;
			spline[RES + 1][2] = 999;
			spline[RES + 1][3] = 999;
			*/
			break;

		case 1:  //Along with the START and GOAL vias there is ONE intermediate point
			vi_1 = 0; //since the GOAL frame has zero velocity

					  //V1 for 2 via0n case (start and goal)
			delta0 = joint[1] - joint[0];
			delta1 = joint[4] - joint[1];

			//h_1 = tau[2] - tau[1];
			h_0 = time[1]; //time to travel from start to intermediate frame, specified by user
			h_1 = time[2];	//time  to travel from intermediate frame to goal, specified by user

			v1 = (3 * delta1*h_0*h_0 + 3 * delta0*h_1*h_1) / (2 * h_0*h_0*h_1 + 2 * h_0*h_1*h_1); //determined through MATLAB


																								  //This method will only work for when we have two points

																								  //Calcuate the discrete values 
			coeff = calcCoeff(joint[0], joint[1], 0, v1, h_0);
			temp = calcDiscreteSpline(0, time[1], coeff);


			//Stich together spline
			for (int i = 0; i < ceil(time[1] / RES); i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i][j];
				}
			}

			coeff = calcCoeff(joint[1], joint[2], v1, v2, h_1);
			temp = calcDiscreteSpline(time[1], time[1] + time[4], coeff);

			//KARA:the result of i=1 needs to be placed on spline after the results of i=0, and so on and so forth

			for (int i = via0; i < numPoints; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via0][j];
				}
			}
			/*		//Make a line of invalid values to represent the end of results
			spline[2 * RES + 1][0] = 999;
			spline[2 * RES + 1][1] = 999;
			spline[2 * RES + 1][2] = 999;
			spline[2 * RES + 1][3] = 999;
			*/
			break;
		case 2:	//There are TWO intermediate points

			delta0 = joint[1] - joint[0];
			delta1 = joint[2] - joint[1];
			delta2 = joint[4] - joint[2];

			h_0 = time[1];
			h_1 = time[2];
			h_2 = time[4];

			v1 = (3 * (-delta2*h_0*h_0*h_1*h_1 + 2 * delta1*h_0*h_0*h_1*h_2 + delta1*h_0*h_0*h_2*h_2 + 2 * delta0*pow(h_1, 3)*h_2 + 2 * delta0*h_1*h_1*h_2*h_2)) /
				(h_0*h_1*h_2*(4 * h_0*h_1 + 3 * h_0*h_2 + 4 * h_1*h_2 + 4 * h_1*h_1));

			v2 = (3 * (2 * delta2*h_0*h_0*h_1*h_1 + delta1*h_0*h_0*h_2*h_2 + 2 * delta2*h_0*pow(h_1, 3) + 2 * delta1*h_0*h_1*h_2*h_2 - delta0*h_1*h_1*h_2*h_2)) /
				(h_0*h_1*h_2*(4 * h_0*h_1 + 3 * h_0*h_2 + 4 * h_1*h_2 + 4 * h_1*h_1));

			//the result of i=1 needs to be placed on spline after the results of i=0, and so on and so forth
			coeff = calcCoeff(joint[0], joint[1], 0, v1, h_0);
			temp = calcDiscreteSpline(0, time[1], coeff);


			for (int i = 0; i < ceil(time[1] / RES); i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i][j];
				}
			}
			coeff = calcCoeff(joint[1], joint[2], v1, v2, h_1);
			temp = calcDiscreteSpline(time[1], tau[0], coeff);

			for (int i = via0; i < via1; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via0][j];
				}
			}
			coeff = calcCoeff(joint[2], joint[3], v2, v3, h_2);
			temp = calcDiscreteSpline(tau[0], tau[0] + time[4], coeff);

			for (int i = via1; i < numPoints; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - 2 * RES][j];
				}
			}

			/*		//Make a line of invalid values to represent the end of results
			spline[3 * RES + 1][0] = 999;
			spline[3 * RES + 1][1] = 999;
			spline[3 * RES + 1][2] = 999;
			spline[3 * RES + 1][3] = 999;
			*/
			break;
		case 3: //There are THREE intermediate points

			delta0 = joint[1] - joint[0];
			delta1 = joint[2] - joint[1];
			delta2 = joint[3] - joint[2];
			delta3 = joint[4] - joint[3];

			h_0 = time[1];// tau[1] - tau[0];
			h_1 = time[2];//tau[2] - tau[1];
			h_2 = time[3];//tau[3] - tau[2];
			h_3 = time[4];//tau[4] - tau[3];

			v1 = (3 * (4 * delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) + 2 * delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - 2 * delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) + delta3*h_0*h_0*h_1*h_1*h_2*h_2 + delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) + 4 * delta0*h_1*h_1*pow(h_2, 3)*h_3 + 3 * delta0*pow(h_1, 3)*h_2*pow(h_3, 2) + 4 * delta0*pow(h_1, 3)*h_2*h_2*h_3 + 2 * delta1*h_0*h_0*pow(h_2, 3)*h_3 + 3 * delta1*h_0*h_0*h_1*h_2*pow(h_3, 2) + 4 * delta1*h_0*h_0*h_1*h_2*h_2*h_3 - 2 * delta2*h_0*h_0*h_1*h_1*h_2*h_3)) /
				(2 * h_0*h_1*h_2*h_3*(3 * h_0*h_2*h_2 + 4 * h_1*h_2*h_2 + 4 * h_1*h_1*h_2 + 3 * h_1*h_1*h_3 + 4 * h_0*h_1*h_2 + 3 * h_0*h_1*h_3 + 3 * h_0*h_2*h_3 + 4 * h_1*h_2*h_3));

			v2 = (3 * (delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) + 2 * delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) - delta3*h_0*h_0*h_1*h_1*h_2*h_2 - delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) - delta0*h_1*h_1*pow(h_2, 3)*h_3 + delta1*h_0*h_0*pow(h_2, 3)*h_3 + 2 * delta2*h_0*pow(h_1, 3)*pow(h_3, 2) - delta3*h_0*pow(h_1, 3)*h_2*h_2 - delta3*h_0*pow(h_1, 3)*pow(h_3, 2) + 2 * delta1*h_0*h_1*h_2*h_2*pow(h_3, 2) + 2 * delta2*h_0*h_0*h_1*h_1*h_2*h_3 + 2 * delta1*h_0*h_1*pow(h_2, 3)*h_3 + 2 * delta2*h_0*pow(h_1, 3)*h_2*h_3)) /
				(h_0*h_1*h_2*h_3*(3 * h_0*h_2*h_2 + 4 * h_1*h_2*h_2 + 4 * h_1*h_1*h_2 + 3 * h_1*h_1*h_3 + 4 * h_0*h_1*h_2 + 3 * h_0*h_1*h_3 + 3 * h_0*h_2*h_3 + 4 * h_1*h_2*h_3));

			v3 = (3 * (delta0*h_1*h_1*h_2*h_2*pow(h_3, 2) - delta1*h_0*h_0*h_2*h_2*pow(h_3, 2) - 2 * delta2*h_0*h_0*h_1*h_1*pow(h_3, 2) + 4 * delta3*h_0*h_0*h_1*h_1*h_2*h_2 + 4 * delta3*h_0*h_0*h_1*h_1*pow(h_3, 2) - 2 * delta2*h_0*pow(h_1, 3)*pow(h_3, 2) + 4 * delta3*h_0*h_1*h_1*pow(h_2, 3) + 4 * delta3*h_0*pow(h_1, 3)*h_2*h_2 + 3 * delta3*h_0*h_0*h_1*pow(h_2, 3) + 4 * delta3*h_0*pow(h_1, 3)*pow(h_3, 2) - 2 * delta1*h_0*h_1*h_2*h_2*pow(h_3, 2) + 4 * delta3*h_0*h_1*h_1*h_2*pow(h_3, 2) + 3 * delta3*h_0*h_0*h_1*h_2*pow(h_3, 2))) /
				(2 * h_0*h_1*h_2*h_3*(3 * h_0*h_2*h_2 + 4 * h_1*h_2*h_2 + 4 * h_1*h_1*h_2 + 3 * h_1*h_1*h_3 + 4 * h_0*h_1*h_2 + 3 * h_0*h_1*h_3 + 3 * h_0*h_2*h_3 + 4 * h_1*h_2*h_3));


			//KARA:the result of coeff[1] needs to be placed on spline after the results of coeff[0], and so on and so forth
			//Kara: Is there a more elgant way to doe this?



			coeff = calcCoeff(joint[0], joint[1], 0, v1, h_0);
			temp = calcDiscreteSpline(0, time[0], coeff);
			for (int i = 0; i < ceil(time[1] / RES); i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i][j];
				}
			}
			coeff = calcCoeff(joint[1], joint[2], v1, v2, h_1);
			temp = calcDiscreteSpline(time[0], tau[0], coeff);

			for (int i = via0; i < via1; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via0][j];
				}
			}

			coeff = calcCoeff(joint[2], joint[3], v2, v3, h_2);
			temp = calcDiscreteSpline(tau[0], tau[1], coeff);

			for (int i = via1; i < via2; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via1][j];
				}
			}
			coeff = calcCoeff(joint[3], joint[4], v3, v4, h_3);
			temp = calcDiscreteSpline(tau[1], tau[2], coeff);

			for (int i = via2; i <via3; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via2][j];
				}
			}
			coeff = calcCoeff(joint[4], joint[5], v4, 0, h_4);
			temp = calcDiscreteSpline(tau[2], tau[2] + time[4], coeff);

			for (int i = via3; i <numPoints; i++) {
				for (int j = 0; j < (WIDTH + 1); j++) {
					spline[i][j] = temp[i - via3][j];
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
	double** traj_plan::calcDiscreteSpline(double t0, double tf, double* coeff) {		//KARA change Time Resolution to a consatant value

		points = (int)ceil((tf - t0) / RES);
		discreteSpline = new double*[points];
		for (int j = 0; j < points; j++) {
			discreteSpline[j] = new double[WIDTH]; //Unhandled excevia0ion at 0x75EE5B68 in RRPR.exe: Microsoft C++ excevia0ion: std::bad_alloc at memory location 0x0073EE20.
		}

		cout << endl << "** Inside calcDiscreteSpline ******" << endl;
		t = t0;
		ii = 0;

		while (t <= tf) {				//Kara: the last entry suposed to be at the final time
			discreteSpline[ii][0] = t;

			discreteSpline[ii][1] = coeff[0] + coeff[1] * t + coeff[2] * t*t + coeff[3] * t*t*t; 
			discreteSpline[ii][2] = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t*t;
			discreteSpline[ii][3] = 2 * coeff[2] + 6 * coeff[3] * t;

			t = t + RES;
			ii++;
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

	int traj_plan::numofPoints(double t1, double t2, double t3, double t4) {


		total_time = t1 + t2 + t3 + t4;
		cout << "Total Time: " << total_time << endl;

		cout << "Number of sample points: " << ceil(total_time / RES) << endl;

		return ceil(total_time / RES);



	}
	void traj_plan::appPrintTrajectory(string filename, double input)
	{
		ofstream myfile(filename, std::ios::app);
		{
			if (input != -9999)
			{
				myfile << input << "\t";
			}
			else
			{
				myfile << endl;
			}

		}
	}
