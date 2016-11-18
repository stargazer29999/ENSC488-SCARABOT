#include "robo_control.h"
#include <chrono>
#include <thread>

using namespace std;

robo_control::robo_control()
{
	JOINT q0 = { 100, 100, -100, 100 };
	OpenMonitor();

	internal_form = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		internal_form[i] = new double[WIDTH];
	}

	user_form = new double[HEIGHT];

	//TMULT
	t_matrix1 = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		t_matrix1[i] = 0;
		t_matrix1[i] = new double[WIDTH];
	}

	t_matrix2 = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		t_matrix2[i] = 0;
		t_matrix2[i] = new double[WIDTH];
	}

	r_matrix = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		r_matrix[i] = new double[WIDTH];
	}

	for (int i = 0; i < HEIGHT; i++) {
		for (int j = 0; j < WIDTH; j++) {
			r_matrix[i][j] = 0;
		}
	}

	vect_5 = new double[5];

	via = new double*[HEIGHT + 1]; //make a 5x5 (row x col)
	for (int i = 0; i < (HEIGHT + 1); i++) {
		via[i] = new double[WIDTH + 1];
	}

	traj = new double **[4];

}

void robo_control::moveJOINT()
{
	cout << endl << "Please enter desired joint values as a list:";
	cin >> user_form[0] >> user_form[1] >> user_form[2] >> user_form[3];
	cout << endl;

	internal_form = cmd.UTOI(user_form);


	cout << "===========================================" << endl;
	cout << "Forward Kinematics Result from WHERE(): " << endl;
	r_matrix = cmd.WHERE(user_form);
	cmd.printMatrix(r_matrix, HEIGHT, WIDTH);

	cout << "The Cartesian coordiantes are :  x= " << (int)r_matrix[0][3] << " y= " << (int)r_matrix[1][3] << "  z= " << (int)r_matrix[2][3] << " PHI= " << (int)(user_form[0] + user_form[1] - user_form[3]) << endl;

	//Error 
	if (!cmd.errorFound(user_form, 1)) {
		JOINT q0 = { user_form[0], user_form[1], user_form[2], user_form[3] };
		DisplayConfiguration(q0);
		//cout << "Joint values are: {" << user_form[0] << " " << user_form[1] << " " << user_form[2] << " " << user_form[3] << "}" << endl;
	}

}

void  robo_control::moveGriper() {
	cout << "Would you like to activate the gripper? Y/N" << endl;
	cin >> grip;

	if (grip == 'Y')
	{
		Grasp(true);
	}
	else {
		Grasp(false);
	}

}

void robo_control::stopRobot()
{
	StopRobot();
	CloseMonitor();
}


void robo_control::initJoint()
{
	JOINT q0 = { 0, 0, -150, 0 };
	Grasp(false);
	DisplayConfiguration(q0);
	cout << "Intialized...";
}

void robo_control::currentJoints()
{
	JOINT q0;
	GetConfiguration(q0);
	cout << "Joint values are : " << q0[0] << " " << q0[1] << " " << q0[2] << " " << q0[3] << endl;
}

void robo_control::currentCartesian()
{
	JOINT q0;
	GetConfiguration(q0);
	r_matrix = cmd.WHERE(q0);
	cout << "The Cartesian coordiantes are :  x= " << r_matrix[0][3] << " y= " << r_matrix[1][3] << " z= " << r_matrix[2][3] << " PHI= " << (q0[0] + q0[1] - q0[3]);
}

void robo_control::zeroPosition()
{
	JOINT q0 = { 90, 0, -175, 0 };
	Grasp(false);
	DisplayConfiguration(q0);
	cout << "Reset Successful" << endl;
}

void robo_control::moveCart()
{
	clock_t cl;     //initializing a clock type

	double dist[2] = { 0, 0 };
	cout << "Please input the desired Cartersian position (as a list): ";// << endl;
	cin >> user_form[0] >> user_form[1] >> user_form[2] >> user_form[3];
	cout << endl;

	JOINT q0;

	bool noSolution = false;
	GetConfiguration(q0);

	r_matrix = cmd.SOLVE(q0, cmd.UTOI(user_form));

	//cout << "Soltution 1. " << (int)r_matrix[0][0] << ", " << (int)r_matrix[0][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;
	//cout << "Soltution 2. " << (int)r_matrix[1][0] << ", " << (int)r_matrix[1][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[1][3] << endl;

	//decided to add int casting on the joint values found from inverse kinematics in case we need more accuracy later
	if (r_matrix[3][3] == 0) {
		JOINT q1 = { (int)r_matrix[1][0], (int)r_matrix[1][1], (int)r_matrix[0][2], (int)r_matrix[0][3] };
		cout << "** No Valid Solution found **" << endl;
		noSolution = true;
		//jump to the end of function
	}
	else if ((int)r_matrix[3][3] == 1) {
		cout << "** First solution is not valid**" << endl;
		cout << "Soltution 2 is chosen: " << (int)r_matrix[1][0] << ", " << (int)r_matrix[1][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[1][3] << endl;

		JOINT q1 = { (int)r_matrix[1][0], (int)r_matrix[1][1], (int)r_matrix[0][2], (int)r_matrix[1][3] };

		cl = clock();   //starting time of clock
		MoveToConfiguration(q1, true);
		cl = clock() - cl;  //end point of clock

		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;
		cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
		cout << "===========================================" << endl;

	}
	else if ((int)r_matrix[3][3] == 2) {
		cout << "** Second solution is not valid**" << endl;
		cout << "Soltution 1 is chosen: " << (int)r_matrix[0][0] << ", " << (int)r_matrix[0][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;

		JOINT q1 = { (int)r_matrix[0][0], (int)r_matrix[0][1], (int)r_matrix[0][2], (int)r_matrix[0][3] };

		cl = clock();   //starting time of clock
		MoveToConfiguration(q1, true);
		cl = clock() - cl;  //end point of clock

		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;
		cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
		cout << "===========================================" << endl;

	}
	else {
		cout << "Two  Solutions found." << endl;
		cout << "Soltution 1. " << (int)r_matrix[0][0] << ", " << (int)r_matrix[0][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;
		cout << "Soltution 2. " << (int)r_matrix[1][0] << ", " << (int)r_matrix[1][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[1][3] << endl;

		if ((int)r_matrix[0][0] != (int)r_matrix[1][0]) {
			cout << "Two different solutions: ";
			//Find the shortest distance if solution is valid
			//sum the metric distances
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 4; j++) {
					dist[i] += abs(q0[j] - r_matrix[i][j]);
				}
			}

			if (dist[0] < dist[1]) {
				cout << "Solution 1 is a shorter distance away" << endl;
				JOINT q1 = { (int)r_matrix[0][0], (int)r_matrix[0][1], (int)r_matrix[0][2], (int)r_matrix[0][3] };
				cl = clock();   //starting time of clock
				MoveToConfiguration(q1, true);
				cl = clock() - cl;  //end point of clock

				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
			else {
				cout << "Solution 2 is a shorter distance away " << endl;
				JOINT q1 = { (int)r_matrix[1][0], (int)r_matrix[1][1], (int)r_matrix[0][2], (int)r_matrix[1][3] };
				cl = clock();   //starting time of clock
				MoveToConfiguration(q1, true);
				cl = clock() - cl;  //end point of clock


				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
		}
		else {
			cout << "Solution 1 & 2 are the same" << endl;
			JOINT q1 = { (int)r_matrix[0][0], (int)r_matrix[0][1], (int)r_matrix[0][2], (int)r_matrix[0][3] };
			cl = clock();   //starting time of clock
			MoveToConfiguration(q1, true);
			cl = clock() - cl;  //end point of clock

			cout << "===========================================" << endl;
			cout << "Forward Kinematics from WHERE(): " << endl;
			cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
			cout << "===========================================" << endl;
		}
	}

	if (noSolution != true)
	{
		cout << "It took " << cl / (double)CLOCKS_PER_SEC << " sec to move robot" << endl;  //prints the determined ticks per second (seconds passed)
	}
}

void robo_control::trajectoryPlan() {
	using namespace std::this_thread; // sleep_for, sleep_until
	using namespace std::chrono; // nanoseconds, system_clock, seconds
	clock_t cl;     //initializing a clock type

	JOINT q0;
	GetConfiguration(q0);
	int numLocations;
	double dist[2] = { 0, 0 };
	int sln;
	bool FAIL = false;
	int samplePoints;


	cout << "The desired START location (x,y,z, phi) ";
	cin >> via[0][0] >> via[0][1] >> via[0][2] >> via[0][3];// >> via[0][4];	//Kara: does the start frame acutally need a time?
	via[0][4] = 999;


	cout << "The desired GOAL location (x,y,z, phi) and time to travel in seconds: " << endl;
	cin >> via[4][0] >> via[4][1] >> via[4][2] >> via[4][3] >> via[4][4];

	cout << "How many intermediate locations (0, 1, 2, 3)? ";
	cin >> numLocations;
	if (numLocations != 0) {
		for (int i = 1; i<4; i++) {  //three or foru?
			if (i <= numLocations) {
				cout << "Intermediate location " << (i) << ". ";
				cin >> via[i][0] >> via[i][1] >> via[i][2] >> via[i][3] >> via[i][4];
				cout << endl;
			}
			else {
				for (int j = 0; j < 5; j++) {
					via[i][j] = 999;	//error code
				}
			}
		}
	}
	else {
		for (int i = 1; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				via[i][j] = 999;	//error code
			}
			via[i][4] = 0;
		}
	}

	cout << "You have input the following locations and time: " << endl;
	cout << setw(15) << "x" << setw(15) << "y" << setw(15) << "z" << setw(15) << "phi" << setw(15) << "time (ms)" << endl;
	cmd.printMatrix(via, (HEIGHT + 1), (WIDTH + 1));


	//Call inverse function for each frame to get desired joint values
	for (int i = 0; i < 5; i++) {
		if (via[i][0] != 999) {	//What it should do: go through calcuations if the value is not 999
			r_matrix = cmd.SOLVE(q0, cmd.UTOI(via[i]));

			if (r_matrix[3][3] == 0) {
				cout << "** No Valid Solution found **" << endl;
			}
			else if (r_matrix[3][3] == 1) {
				cout << "** First solution is not valid**" << endl;
				for (int j = 0; j < 4; j++) {
					via[i][j] = r_matrix[1][j];
				}
			}
			else if (r_matrix[3][3] == 2) {
				cout << "** Second solution is not valid**" << endl;
				for (int j = 0; j < 4; j++) {
					via[i][j] = r_matrix[0][j];
				}
			}
			else {
				if (r_matrix[0][0] != r_matrix[1][0]) {
					for (int i = 0; i < 2; i++) {
						for (int j = 0; j < 4; j++) {
							dist[i] += abs(q0[j] - r_matrix[i][j]);
						}
					}
					if (dist[0] > dist[1]) {
						sln = 0;
					}
					else {
						sln = 1;
					}
				}
				else {
					sln = 0;
				}
				for (int j = 0; j < 4; j++) {
					via[i][j] = r_matrix[sln][j];
				}
			}
		}
	}


	cout << "You have input the following joint values and time to move to: " << endl;
	cout << setw(15) << "j1" << setw(15) << "j2" << setw(15) << "j3" << setw(15) << "j4" << setw(15) << "time (ms)" << endl;
	cmd.printMatrix(via, (HEIGHT + 1), (WIDTH + 1));


	//Move to the Starting point
	JOINT q1 = { via[0][0], via[0][1], via[0][2], via[0][3] };
	MoveToConfiguration(q1, false);

	//Get number of data points
	samplePoints = cmd2.numofPoints(via[1][4], via[2][4], via[3][4], via[4][4]);

	//Allocate space for trajectory
	for (int i = 0; i < (samplePoints); i++) {
		traj[i] = new double*[WIDTH];
		for (int j = 0; j < WIDTH; j++) {
			traj[i][j] = new double[WIDTH];
		}
	}

	//Call trajectory planning function
	traj = cmd2.discreteTrajectory(via, numLocations, samplePoints);

	ii = 0;
	cl = clock();   //starting time of clock

	while (ii<samplePoints && (int)traj[0][ii][0] != 999){
		//for (int ii = 0; ii < 4 * RES; ii++) {
		for (int j = 1; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				user_form[k] = traj[k][ii][j];
			}
			//	if (cmd.errorFound(traj[][ii][1], 1) || cmd.errorFound(traj[][ii][2], 2) || cmd.errorFound(traj[][ii][3], 3)) {	//KARA QUESTION: Does this produce valid results?
			if (cmd.errorFound(user_form, j )) {	//KARA QUESTION: Does this produce valid results?
				//cout << "ERROR FOUND" << endl;
				FAIL = true;
			}
		}
		if (!FAIL) {

			JOINT q0 = { traj[0][ii][1], traj[1][ii][1], traj[2][ii][1], traj[3][ii][1] };
			JOINT q1 = { traj[0][ii][2], traj[1][ii][2], traj[2][ii][2], traj[3][ii][2] };
			JOINT q2 = { traj[0][ii][3], traj[1][ii][3], traj[2][ii][3], traj[3][ii][3] };
			MoveWithConfVelAcc(q0, q1, q2); //Kara Question: How to get the program to pause (time resoltuion delta t) before reading the next value?
			sleep_for(milliseconds(10));
			
			//add sleep here.
			//sleep in sec.
			//_sleep(traj[0][ii + 1][0] - traj[0][ii][0]);
			//std::this_thread::sleep_for(std::chrono::milliseconds((traj[0][ii + 1][0] - traj[0][ii][0])));
			//final - initial time comparison
			//Pause here
		}
		ii++;
	}
	stopRobot();
	//ADD: end timer here and print result to compare to sum of input timer  values
	cl = clock() - cl;  //end point of clock
	cout << "It took " << cl / (double)CLOCKS_PER_SEC << " sec to move robot" << endl;  //prints the determined ticks per second (seconds passed)

}

/*
void robo_control::getStateID()
{
JOINT q0;
GetConfiguration(q0);
bool returnedVal = GetState(q0);
if (returnedVal == true)
{
cout << "worked";
std::stringstream cout;
cout << std::hex << (long)q0[0];
std::string result(cout.str())
cout << (long)q0[0] << " " << (long)q0[1] << " " << (long)q0[2] << " " << (long)q0[3] << endl;
}
else
{
cout << "failed";
}
}
*/
