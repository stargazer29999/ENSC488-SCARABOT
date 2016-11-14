#include "ui_console.h"
#include "mat_kin.h"

using namespace std;

void main()
{
	mat_kin test;
/*	double* user_form;
	user_form = new double[HEIGHT];

	double** internal_form;
	internal_form = new double*[HEIGHT];
	for (int i = 0; i < HEIGHT; i++) {
		internal_form[i] = new double[WIDTH];
	}

	for (int i = 0; i < 3; i++){
		user_form[i] = 0;
	}

	internal_form = test.UTOI(user_form);

	test.printInternalMatrix(internal_form);
*/	
	ui_console cmd;
	cmd.welcomeSplash();
}