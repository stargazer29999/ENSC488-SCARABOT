#include "robo_control.h"
#include <chrono>
//#include <thread>

using namespace std;

robo_control::robo_control()
{
	JOINT q0 = { 90, 0, -175, 0 };
	OpenMonitor();
	
}

void robo_control::moveJOINT()
{
	cout << endl << "Please enter desired joint values as a list:";
	cin >> j1 >> j2 >> j3 >> j4;

	user_form.push_back(j1);
	user_form.push_back(j2);
	user_form.push_back(j3);
	user_form.push_back(j4);

	cout << endl;

	//	internal_form = cmd.UTOI(user_form);
	cout << "===========================================" << endl;
	cout << "Forward Kinematics Result from WHERE(): " << endl;
	r_matrix = cmd.WHERE(user_form);
	cmd.printMatrix(r_matrix, HEIGHT, WIDTH);

	cout << "The Cartesian coordiantes are :  x= " << (int)r_matrix[4*0+3] << " y= " << (int)r_matrix[4*1+3] << "  z= " << (int)r_matrix[4*2+3] << " PHI= " << (int)(user_form[0] + user_form[1] - user_form[3]) << endl;

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
	//CloseMonitor();//our current stpo robot function closes monitor as well

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

	for (int i = 0; i < 4; i++) {
		vectQ0[i] = q0[i];
	}

	r_matrix = cmd.WHERE(vectQ0);
	cout << "The Cartesian coordiantes are :  x= " << r_matrix[0+3] << " y= " << r_matrix[4*1+3] << " z= " << r_matrix[4*2+3] << " PHI= " << (q0[0] + q0[1] - q0[3]);
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

	for (int i = 0; i < 4; i++) {
		vectQ0[i] = q0[i];
	}

	
	r_matrix = cmd.SOLVE(vectQ0, cmd.UTOI(user_form));

	//decided to add int casting on the joint values found from inverse kinematics in case we need more accuracy later
	if ((int)r_matrix[4*3+3] == 0) 
	{
		JOINT q1 = { round(r_matrix[4*1+0]), round(r_matrix[4*1+1]), round(r_matrix[4*0+2]), round(r_matrix[4*0+3])};
		cout << "** No Valid Solution found **" << endl;
		noSolution = true;
		//jump to the end of function
	}
	else if ((int)r_matrix[4*3+3] == 1) {
		cout << "** First solution is not valid**" << endl;
		cout << "Soltution 2 is chosen: " << round(r_matrix[4*1+0]) << ", " << round(r_matrix[4*1+1]) << ", " << round(r_matrix[4*0+2]) << ", " << round(r_matrix[4*1+3]) << endl;

		JOINT q1 = { round(r_matrix[4*1+0]), round(r_matrix[4*1+1]), round(r_matrix[4*0+2]), round(r_matrix[4*1+3]) };

		cl = clock();   //starting time of clock
		MoveToConfiguration(q1, true);
		cl = clock() - cl;  //end point of clock

		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;

		for (int i = 0; i < 4; i++) {
			vectQ1[i] = q1[i];
		}

		r_matrix = cmd.WHERE(vectQ1);

		cmd.printMatrix(r_matrix, HEIGHT, WIDTH);
		cout << "===========================================" << endl;

	}
	else if ((int)r_matrix[4*3+3] == 2) {
		cout << "** Second solution is not valid**" << endl;
		cout << "Soltution 1 is chosen: " << (int)r_matrix[4*0+0] << ", " << (int)r_matrix[4*0+1] << ", " << (int)r_matrix[4*0+2] << ", " << (int)r_matrix[4*0+3] << endl;

		JOINT q1 = { (int)r_matrix[4*0+0], (int)r_matrix[4*0+1], (int)r_matrix[4*0+2], (int)r_matrix[4*0+3] };

		cl = clock();   //starting time of clock
		MoveToConfiguration(q1, true);
		cl = clock() - cl;  //end point of clock

		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;

		for (int i = 0; i < 4; i++) {
			vectQ1[i] = q1[i];
		}

		r_matrix = cmd.WHERE(vectQ1);

		cmd.printMatrix(r_matrix, HEIGHT, WIDTH);
		cout << "===========================================" << endl;

	}
	else {
		cout << "Two  Solutions found." << endl;
		cout << "Soltution 1. " << (int)r_matrix[4*0+0] << ", " << (int)r_matrix[4*0+1] << ", " << (int)r_matrix[4*0+2] << ", " << (int)r_matrix[4*0+3] << endl;
		cout << "Soltution 2. " << (int)r_matrix[4*1+0] << ", " << (int)r_matrix[4*1+1] << ", " << (int)r_matrix[4*0+2] << ", " << (int)r_matrix[4*1+3] << endl;

		if ((int)r_matrix[4*0+0] != (int)r_matrix[4*1+0]) {
			cout << "Two different solutions: ";
			//Find the shortest distance if solution is valid
			//sum the metric distances
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 4; j++) {
					dist[i] += abs(q0[j] - r_matrix[i*j]);
				}
			}
			if (dist[0] < dist[1]) {
				cout << "Solution 1 is a shorter distance away" << endl;
				JOINT q1 = { (int)r_matrix[4*0+0], (int)r_matrix[4*0+1], (int)r_matrix[4*0+2], (int)r_matrix[4*0+3] };
				cl = clock();   //starting time of clock
				MoveToConfiguration(q1, true);
				cl = clock() - cl;  //end point of clock

				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;

				for (int i = 0; i < 4; i++) {
					vectQ1[i] = q1[i];
				}

				r_matrix = cmd.WHERE(vectQ1);

				cmd.printMatrix(r_matrix, HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
			else {
				cout << "Solution 2 is a shorter distance away " << endl;
				JOINT q1 = { (int)r_matrix[4*1+0], (int)r_matrix[4*1+1], (int)r_matrix[4*0+2], (int)r_matrix[4*1+3] };
				cl = clock();   //starting time of clock
				MoveToConfiguration(q1, true);
				cl = clock() - cl;  //end point of clock

				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;

				for (int i = 0; i < 4; i++) {
					vectQ1[i] = q1[i];
				}

				r_matrix = cmd.WHERE(vectQ1);

				cmd.printMatrix(r_matrix, HEIGHT, WIDTH);
				cout << "===========================================" << endl;
			}
		}
		else {
			cout << "Solution 1 & 2 are the same" << endl;
			JOINT q1 = { (int)r_matrix[4*0+0], (int)r_matrix[4*0+1], (int)r_matrix[4*0+2], (int)r_matrix[4*0+3] };
			cl = clock();   //starting time of clock
			MoveToConfiguration(q1, true);
			cl = clock() - cl;  //end point of clock

			cout << "===========================================" << endl;
			cout << "Forward Kinematics from WHERE(): " << endl;

			for (int i = 0; i < 4; i++) {
				vectQ1[i] = q1[i];
			}

			r_matrix = cmd.WHERE(vectQ1);

			cmd.printMatrix(r_matrix, HEIGHT, WIDTH);
			cout << "===========================================" << endl;
		}
	}

	if (noSolution != true)
	{
		cout << "It took " << cl / (double)CLOCKS_PER_SEC << " sec to move robot" << endl;  //prints the determined ticks per second (seconds passed)
	}
}


//needs work here *tried to implement movewithvelAcc function - Jason
void robo_control::moveWithVelAcc(JOINT& q1, JOINT& q2, JOINT& q3)
{
	MoveWithConfVelAcc(q1,q2,q3);
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

void robo_control::RESETROBOT() {
	ResetRobot();
}