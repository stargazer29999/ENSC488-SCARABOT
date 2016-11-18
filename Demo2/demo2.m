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

close all
%http://www.mathworks.com/help/matlab/ref/fscanf.html 
    fileID2 = fopen('numPoints.txt','r');
    formatSpec = '%f';
    numPoints=fscanf(fileID2,formatSpec, 1);
    fclose(fileID2);
    numPoints=numPoints'


%open the file for reading, and obtain the file identifier, fileID.
    fileID1 = fopen('Joints.txt','r');
    
    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    Joints = fscanf(fileID1,formatSpec, jointSize);
    fclose(fileID1);   

    %Transpose the array so that * matches the orientation of the data in the file.
    Joints = Joints'
    %%
    joint1=Joints(1:numPoints+1,:)
    joint2=Joints((numPoints+2):2*numPoints+2,:)
    joint3=Joints((2*numPoints+3):3*numPoints+3,:)
    joint4=Joints((3*numPoints+4):4*numPoints+4,:)
 

%% Handle Data for the first joint
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint1(:,1),joint1(:,2),'-*k')
	title(ax1,strcat('Theta ','1'))

	ylabel(ax1,strcat('Theta ','1'))
	xlabel(ax1,'time(ms)' )

	plot(ax2,joint1(:,1),joint1(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(ms)' )

	plot(ax3,joint1(:,1),joint1(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(ms)' )


%% Handle Data for the Second joint
	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint2(:,1),joint2(:,2),'-*k')
	title(ax1,strcat('Theta ','2'))close

	ylabel(ax1,strcat('Theta ','2'))
	xlabel(ax1,'time(ms)' )

	plot(ax2,joint2(:,1),joint2(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(ms)' )

	plot(ax3,joint2(:,1),joint2(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(ms)' )



%% Handle Data for the third joint
	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint3(:,1),joint3(:,2),'-*k')
	title(ax1,strcat('Theta ','3'))

	ylabel(ax1,strcat('Theta ','3'))
	xlabel(ax1,'time(ms)' )

	plot(ax2,joint3(:,1),joint3(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(ms)' )

	plot(ax3,joint3(:,1),joint3(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(ms)' )



%% Handle Data for the fourth joint
	%Plot Data
	figure % new figure
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint4(:,1),joint4(:,2),'-*k')
	title(ax1,strcat('Theta ','4'))

	ylabel(ax1,strcat('Theta ','4'))
	xlabel(ax1,'time(ms)' )

	plot(ax2,joint4(:,1),joint4(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(ms)' )

	plot(ax3,joint4(:,1),joint4(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(ms)' )
