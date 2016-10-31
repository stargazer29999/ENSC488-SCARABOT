function Theta4=findTheta4(Tmatrix,theta1,theta2)
    c = cos(theta1)*Tmatrix(1, 2) + Tmatrix(2, 2)*sin(theta1);
	a = sin(theta2);
	b = cos(theta2);
	if ((c + a) == 0) 
		disp('Degenerate case\n');
		theta4 = 2 * atan2(-a / b, 1)
        theta4_=NaN %???
		disp('apply for both theta2 and theta2** for a total of 2 cases');
    elseif (b == 0)
			disp('Degenerate case\n');
        if (c == 0)
            
            disp('infinite solutions\n');
        else
            theta4=NaN 
            theta4_=NaN 
            disp('no solution\n');

        end
		
    else
		theta4 = atan2((b + sqrt(b*b - c*c + a*a)) / (c + a), 1)
		theta4_= atan2((b - sqrt(b*b - c*c + a*a)) / (c + a), 1)
		disp('apply for both theta2 and theta2** for a total of four cases');
    end
    Theta4=[theta4, theta4_]
    end