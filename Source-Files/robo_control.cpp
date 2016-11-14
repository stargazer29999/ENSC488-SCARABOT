#include "robo_control.h"

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


	via = new double*[HEIGHT+1]; //make a 5x4 (row x col)
	for (int i = 0; i < (HEIGHT+1); i++) {
		via[i] = new double[WIDTH];
	}
}

void robo_control::moveJOINT()
{
	cout << endl << "Please enter desired joint values as a list:";
	cin >> user_form[0] >> user_form[1] >> user_form[2]  >> user_form[3];
	cout << endl;

	internal_form = cmd.UTOI(user_form);

	
	cout << "===========================================" << endl;
	cout << "Forward Kinematics Result from WHERE(): " << endl;
	r_matrix = cmd.WHERE(user_form);
	cmd.printMatrix(r_matrix,HEIGHT,WIDTH);

	cout << "The Cartesian coordiantes are :  x=" << (int)r_matrix[0][3] << " y=" << (int)r_matrix[1][3] << "  z=" << (int)r_matrix[2][3] << " PHI=" << (int)(acos(r_matrix[0][0]) * 180 / PI) << endl;

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

	cout << "The Cartesian coordiantes are :  x= " << r_matrix[0][3] << " y= " << r_matrix[1][3] << " z= " << r_matrix[2][3] << " PHI= " <<(int)( atan2(r_matrix[1][3], r_matrix[0][3]) * 180 / PI);
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
	double dist[2] = { 0,0 };
	cout << "Please input the desired Cartersian position (as a list): ";// << endl;
	cin >> user_form[0] >> user_form[1] >> user_form[2]>> user_form[3];
	cout << endl;

	JOINT q0;
	GetConfiguration(q0);

	r_matrix = cmd.SOLVE(q0, cmd.UTOI(user_form));

	if(r_matrix[3][3]==0){
		JOINT q1 = { r_matrix[1][0], r_matrix[1][1] ,r_matrix[0][2], r_matrix[0][3] };
		cout << "** No Valid Solution found **" << endl;

		cout << "Soltution 1. " << (int)r_matrix[0][0] << ", " << (int)r_matrix[0][1]  << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;
		cout << "Soltution 2. " << (int)r_matrix[1][0] << ", " << (int)r_matrix[1][1]  << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;

		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;
		cmd.printMatrix(cmd.WHERE(q1),HEIGHT, WIDTH);
		cout << "===========================================" << endl;
	}
	else {

		cout<<"Two  Solutions" << endl;
		cout << "Soltution 1. " << (int)r_matrix[0][0] << ", " << (int)r_matrix[0][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;
		cout << "Soltution 2. " << (int)r_matrix[1][0] << ", " << (int)r_matrix[1][1] << ", " << (int)r_matrix[0][2] << ", " << (int)r_matrix[0][3] << endl;

		if (r_matrix[0][0] != r_matrix[1][0]) {
			cout << "Two different solutions: ";
			//Find the shortest distance if solution is valid
			//sum the metric distances
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 4; j++) {
					dist[i]+=abs(q0[j] - r_matrix[i][j]);
				}
			}
			if (dist[0] > dist[1]) {
				JOINT q1 = { r_matrix[0][0], r_matrix[0][1] ,r_matrix[0][2], r_matrix[0][3] };
				MoveToConfiguration(q1, false);
				cout << "Solution 1 is a shorter distance away" << endl;
				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
			else {
				JOINT q1 = { r_matrix[1][0], r_matrix[1][1], r_matrix[0][2], r_matrix[0][3] };
				MoveToConfiguration(q1, false);
				cout << "Solution 2 is a shorter distance away " << endl;
				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
		}
		else {
			JOINT q1 = { r_matrix[0][0], r_matrix[0][1], r_matrix[0][2], r_matrix[0][3] };
			MoveToConfiguration(q1, false);
			cout << "===========================================" << endl;
			cout << "Forward Kinematics from WHERE(): " << endl;
			cmd.printMatrix(cmd.WHERE(q1), HEIGHT, WIDTH);
			cout << "===========================================" << endl;
		}
	}
	
}

void robo_control::trajectoryPlan() {
	
	JOINT q0;
	DisplayConfiguration(q0);
	int numLocations;
	double dist[2] = { 0,0 };
	int sln;
	bool FAIL = false;


	cout << "The desired START location (x,y,z, phi): ";
	cin >> via[0][0] >> via[0][1] >> via[0][2] >> via[0][3];

	cout << "The desired GOAL location (x,y,z, phi): ";
	cin >> via[4][0] >> via[4][1] >> via[4][2] >> via[4][3];

	cout << "How many intermediate locations (0, 1, 2, 3)? ";
	cin >> numLocations;
	if (numLocations != 0) {
		for (int i=1; i<4; i++){  //three or foru?
			if (i <= numLocations) {
				cout << "Please input the desired intermediate location (x,y,z, phi): ";
				cin >> via[i][0] >> via[i][1] >> via[i][2] >> via[i][3];
				cout << endl;
			}
			else {
				for (int j = 0; j < 4; j++) {
					via[i][j] = -999;	//error code
				}
			}
		}
	}

	//Comment out the follwoing section once the code works to save space
	cout << "You have input the following location: " << endl;
	cout << setw(15)<<"x"<<setw(15)<< "y"<< setw(15)<<"z"<< setw(15)<<"phi"<<endl;
	cmd.printMatrix(via,(HEIGHT + 1), WIDTH);
	//end of comment out section


	//Call inverse function for each frame to get desired joint values
	for (int i = 0; i < 5; i++) {
		if (via[i][0] != -999) {	//What it should do: go through calcuations if the value is not -999
			r_matrix = cmd.SOLVE(q0, cmd.UTOI(via[i]));

			if (r_matrix[3][3] == 0) {
				cout << "** No Valid Solution found **" << endl;
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
					via[i][j]= r_matrix[sln][j];
				}
			}
		}
	}
	//Call trajectory planning function -> change to return something
	//traj, a matrix of __x__x_ which holds the values 
	traj=cmd2.discreteTrajectory(via,numLocations);
	for (int i = 0; i < TIME; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				user_form[k] = traj[k][i][j];
			}
		//	if (cmd.errorFound(traj[][i][1], 1) || cmd.errorFound(traj[][i][2], 2) || cmd.errorFound(traj[][i][3], 3)) {	//KARA QUESTION: Does this produce valid results?
			if (j<2&&cmd.errorFound(user_form, (j+1))) {	//KARA QUESTION: Does this produce valid results?
				cout << "ERROR FOUND" << endl;	
				FAIL = true;
			}
		}
		if(!FAIL) {
			
			JOINT q0 = { traj[0][i][1], traj[2][i][1] , traj[3][i][1], traj[4][i][1] };
			JOINT q1 = { traj[0][i][2], traj[2][i][2] , traj[3][i][2], traj[4][i][2] };
			JOINT q2= { traj[0][i][3], traj[2][i][3] , traj[3][i][3], traj[4][i][3] };
			MoveWithConfVelAcc(q0, q1, q2); //Kara Question: How to get the program to pause (time resoltuion delta t) before reading the next value?
		}
	}
	


}