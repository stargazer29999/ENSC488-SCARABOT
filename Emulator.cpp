#include <conio.h>
#include "ensc-488.h"

int main(int argc, char* argv[])
{
	//joint variables
	JOINT q1 = { 0, 0, -100, 0 };
	JOINT q2 = { 90, 90, -200, 45 };
	

	char ch;
	do
	{
		// _getch() function waits for user's key entries and stores it to variable ch
		ch = _getch();

		//if 1 is pressed execute the functions in if-else statement
		if (ch == '1')
		{
			//DisplayConfiguration(q1);
			
			//move to given configuration q2 then to q1
			MoveToConfiguration(q2, false);
			MoveToConfiguration(q1, false);
		}
		//if 2 is pressed execute the functions in if-else statement
		else if (ch == '2')
		{
			//DisplayConfiguration(q2);

			//move to given configuration q1 then to q2
			MoveToConfiguration(q1, false);
			MoveToConfiguration(q2, false);
		}
	} while (ch != 27);	// 27 refers to character ESC key to exit the do-while loop
	
	return 0;
}