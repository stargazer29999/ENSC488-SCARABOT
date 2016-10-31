function internal_form= testKIN(joint)

	theta1 = joint(1) * (pi() / 180);
	theta2 = joint(2) * (pi() / 180);
	d3 = joint(3);
	theta4 = joint(4) * (pi() / 180);


internal_form=zeros(4);

internal_form(1, 1) = sin(theta4)*sin(theta1 + theta2) - cos(theta4)*cos(theta1 + theta2);
internal_form(1, 2) = sin(theta1 + theta2 + theta4);
internal_form(1, 3) = 0;
internal_form(1, 4) = 195 * cos(theta1) + 142 * cos(theta1 + theta2);

internal_form(2, 1) = -sin(theta1 + theta2 + theta4);
internal_form(2, 2) = sin(theta4)*sin(theta1 + theta2) - cos(theta4)*cos(theta1 + theta2);
internal_form(2, 3) = 0;
internal_form(2, 4) = 195 * sin(theta1) + 142 * sin(theta1*theta2);

internal_form(3, 1) = 0;
internal_form(3, 2) = 0;
internal_form(3, 3) = -1;
internal_form(3, 4) = -d3 - 480;

internal_form(4, 1) = 0;
internal_form(4, 2) = 0;
internal_form(4, 3) = 0;
internal_form(4, 4) = 1;

end