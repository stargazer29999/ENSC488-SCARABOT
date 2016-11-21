
#include "ui_console.h"
#include "ensc-488.h"

void ui_console::welcomeSplash()
{
/*	//cout << "ENSC 488 - Introduction to Robotics: Demo 1 w/ WIP UI" << endl; add to main
	cout << "Press '1' for emulator " << endl; // '0' for hw communcation" << endl;
	cout << "Input: ";
	cin >> select_bit;

	while (select_bit != 1 || select_bit != 0)
	{
		if (select_bit == 1)
		{
	*/		emulatorSplash();
/*		}
		else if (select_bit == 0)
		{
			//hwSplash();
		}
		else
		{
			welcomeSplash();
		}
	}
	*/
}

void ui_console::emulatorSplash()
{
	StopRobot();
	ResetRobot();

	cout << " " << endl;
	cout << "List of Commands, please select one by typeing corresponding command number" << endl;
	cout << "0: Quit Emulation" << endl;
	cout << "1: Initialize Robot Position, joints = (0,0,-150,0)" << endl;
	cout << "2: Reset to Start Position, joints = (90,0,-175,0)" << endl;
	cout << "3: Set Joint Values (WHERE function)" << endl;
	cout << "4: Move to X,Y,Z, Phi (SOLVE function)" << endl;
	cout << "5: Move Gripper" << endl;
	cout << "6: Get Current Joint Configuration" << endl;
	cout << "7: Get Current Cartesian Coordinates " << endl;
	cout << "8: Plan a Robot Trajectory" << endl; 							//KARA
	cout << "Input: ";
	
	emul_select = -1;
	//cout << "" << endl;

	while (emul_select != 0 || emul_select != 1 || emul_select != 2 || emul_select != 2 || emul_select != 3 || emul_select != 4)
	{
		cin >> emul_select;
		cout << endl;
		switch (emul_select)
		{
		case 0: //done
			quitSplash();
			exit(-1);
			break;
		case 1:
			cmd.initJoint();
			break;
		case 2:
			cmd.zeroPosition();
			break;
		case 3:
			cmd.moveJOINT();
			break;
		case 4:
			cmd.moveCart();
			break;
		case 5:
			cmd.moveGriper();
			break;
		case 6:
			cmd.currentJoints();			
			break;
		case 7:
			cmd.currentCartesian();
			break;
		case 8:							//KARA
			//cmd.trajectoryPlan();		//KARA
			break;						//KARA
		default:
			quitSplash();
			break;
		}
		emulatorSplash();
	}
}

void ui_console::quitSplash()
{
	cout << "System shutting down..." << endl;
	//necceasy reset command if any
	cout << "" << endl;
	cmd.stopRobot();
	system("pause");
}
