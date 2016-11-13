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
	
}

void robo_control::moveJOINT()
{
	cout << endl << "Please enter desired joint values" << endl;
	cout << "Joint1: ";
	cin >> user_form[0];
	cout << "Joint2: ";
	cin >> user_form[1];
	cout << "Joint3: ";
	cin >> user_form[2]; 
	cout << "Joint4: ";
	cin >> user_form[3];
	cout << endl;

	internal_form = cmd.UTOI(user_form);

	//Error 
	if (user_form[0] > 150 || user_form[0] < -150) {
		cout << "ERROR: Joint 1 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (user_form[1] > 100 || user_form[1] < -100) {
		cout << "ERROR: Joint 2 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (user_form[2] <= -200 || user_form[2]>-100) {
		cout << "ERROR: Joint 3 limit\n CHANGE code in GUI to Exit Program";
	}
	else if (user_form[3] > 160 || user_form[3] <= -160) {
		cout << "ERROR: Joint 4 limit\n CHANGE code in GUI to Exit Program";
	}
	else {

		JOINT q0 = { user_form[0], user_form[1], user_form[2], user_form[3] };
		MoveToConfiguration(q0, false);

		//cout << "Joint values are: {" << user_form[0] << " " << user_form[1] << " " << user_form[2] << " " << user_form[3] << "}" << endl;
		cout << "===========================================" << endl;
		cout << "Forward Kinematics Result from WHERE(): " <<  endl;
		r_matrix = cmd.WHERE(user_form);
		cmd.printMatrix(r_matrix);

		cout << "The Cartesian coordiantes are :  x=" << (int)r_matrix[0][3] << " y=" << (int)r_matrix[1][3] << "  z=" << (int)r_matrix[2][3] << " PHI=" << (int)(acos(r_matrix[0][0]) * 180 / PI) << endl;
		

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
	cout << "Please input the desired Cartersian position." << endl;
	cout << "X: ";
	cin >> user_form[0];
	cout << "Y: ";
	cin >> user_form[1];
	cout << "Z: ";
	cin >> user_form[2];
	cout << "Phi: ";
	cin >> user_form[3];
	cout << endl;

	JOINT q0;
	GetConfiguration(q0);

	r_matrix = cmd.SOLVE(q0, cmd.UTOI(user_form));

	if(r_matrix[3][3]==0){
		JOINT q1 = { r_matrix[1][0], r_matrix[1][1] ,r_matrix[0][2], r_matrix[0][3] };
		cout << "** No Valid Solution found **" << endl;

		cout << " joint 1: " << (int)r_matrix[0][0] << " joint 2: " << (int)r_matrix[0][1] << endl;
		cout << " joint 1: " << (int)r_matrix[1][0] << " joint 2: " << (int)r_matrix[1][1] << endl;
		cout << " joint 3: " << (int)r_matrix[0][2] << " joint 4: " << (int)r_matrix[0][3] << endl;
		cout << "===========================================" << endl;
		cout << "Forward Kinematics from WHERE(): " << endl;
		cmd.printMatrix(cmd.WHERE(q1));
		cout << "===========================================" << endl;
	}
	else {

		cout<<"Two  Solutions" << endl;
		cout << " joint 1: " << (int)r_matrix[0][0] << " joint 2: "<< (int)r_matrix[0][1] << endl;
		cout << " joint 1: " << (int)r_matrix[1][0] << " joint 2: "<< (int)r_matrix[1][1] << endl;
		cout << " joint 3: " << (int)r_matrix[0][2] << " joint 4: "<< (int)r_matrix[0][3] << endl;

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
				cout << "the first solution is a shorter distance away" << endl;
				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1));
				cout << "===========================================" << endl;
			}
			else {
				JOINT q1 = { r_matrix[1][0], r_matrix[1][1], r_matrix[0][2], r_matrix[0][3] };
				MoveToConfiguration(q1, false);
				cout << "the second solution is a shorter distance away " << endl;
				cout << "===========================================" << endl;
				cout << "Forward Kinematics from WHERE(): " << endl;
				cmd.printMatrix(cmd.WHERE(q1));
				cout << "===========================================" << endl;
			}
		}
		else {
			JOINT q1 = { r_matrix[0][0], r_matrix[0][1], r_matrix[0][2], r_matrix[0][3] };
			MoveToConfiguration(q1, false);
			cout << "===========================================" << endl;
			cout << "Forward Kinematics from WHERE(): " << endl;
			cmd.printMatrix(cmd.WHERE(q1));
			cout << "===========================================" << endl;
		}
	}
	
}

void robo_control::trajectoryPlan() {








}