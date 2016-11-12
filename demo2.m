%Get Matlab to read text file
%output: graphs of each joint
% Matlab must point to the correct file location

%Read Files
 %joint1= readFile('joint1.txt')
%joint2 = readFile('joint2.txt')
%joint3 = readFile('joint3.txt')
%joint4 = readFile('joint4.txt')

%plotJoints(joint1, '1')
%plotJoints(joint2, '2')
%plotJoints(joint3, '3')
%plotJoints(joint4, '4')

%% Handle Data for the first joint
%http://www.mathworks.com/help/matlab/ref/fscanf.html


    %open the file for reading, and obtain the file identifier, fileID.
    fileID = fopen('joint1.txt','r');

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    joint1 = fscanf(fileID,formatSpec, jointSize);
    fclose(fileID);

    %Transpose the array so that * matches the orientation of the data in the file.
    joint1 = joint1';


	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint1(:,1),joint1(:,2))
	title(ax1,strcat('Theta ','1'))

	ylabel(ax1,strcat('Theta ','1'))
	xlabel(ax1,'time(UNITS??)' )

	plot(ax2,joint1(:,1),joint1(:,3))
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(UNITS??)' )

	plot(ax3,joint1(:,1),joint1(:,4))
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(UNITS??)' )



%% Handle Data for the Second joint
%http://www.mathworks.com/help/matlab/ref/fscanf.html


    %open the file for reading, and obtain the file identifier, fileID.
    fileID = fopen('joint2.txt','r');

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    joint2 = fscanf(fileID,formatSpec, jointSize);
    fclose(fileID);

    %Transpose the array so that * matches the orientation of the data in the file.
    joint2 = joint2';


	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint2(:,1),joint2(:,2))
	title(ax1,strcat('Theta ','2'))

	ylabel(ax1,strcat('Theta ','2'))
	xlabel(ax1,'time(UNITS??)' )

	plot(ax2,joint2(:,1),joint2(:,3))
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(UNITS??)' )

	plot(ax3,joint2(:,1),joint2(:,4))
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(UNITS??)' )



%% Handle Data for the third joint

%http://www.mathworks.com/help/matlab/ref/fscanf.html


    %open the file for reading, and obtain the file identifier, fileID.
    fileID = fopen('joint3.txt','r');

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    joint3 = fscanf(fileID,formatSpec, jointSize);
    fclose(fileID);

    %Transpose the array so that * matches the orientation of the data in the file.
    joint3 = joint3';


	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint3(:,1),joint3(:,2))
	title(ax1,strcat('Theta ','3'))

	ylabel(ax1,strcat('Theta ','3'))
	xlabel(ax1,'time(UNITS??)' )

	plot(ax2,joint3(:,1),joint3(:,3))
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(UNITS??)' )

	plot(ax3,joint3(:,1),joint3(:,4))
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(UNITS??)' )



%% Handle Data for the fourth joint

%http://www.mathworks.com/help/matlab/ref/fscanf.html

    %open the file for reading, and obtain the file identifier, fileID.
    fileID = fopen('joint4.txt','r');

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    joint4 = fscanf(fileID,formatSpec, jointSize);
    fclose(fileID);

    %Transpose the array so that * matches the orientation of the data in the file.
    joint4 = joint4';


	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint4(:,1),joint4(:,2))
	title(ax1,strcat('Theta ','4'))

	ylabel(ax1,strcat('Theta ','4'))
	xlabel(ax1,'time(UNITS??)' )

	plot(ax2,joint4(:,1),joint4(:,3))
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(UNITS??)' )

	plot(ax3,joint4(:,1),joint4(:,4))
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(UNITS??)' )



