%Get Matlab to read text file
%output: graphs of each joint
% Matlab must point to the correct file location

%Read Files

close all
%http://www.mathworks.com/help/matlab/ref/fscanf.html 
    fileID2 = fopen('numPoints.txt','r');
    formatSpec = '%f';
    numPoints=fscanf(fileID2,formatSpec, 1);
    fclose(fileID2);
    numPoints=numPoints'


%open the file for reading, and obtain the file identifier, fileID.
    fileID1 = fopen('Joints.txt','r');
    % this file contains information in columns of 
    %[time x y z phi]
    
    
    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f %f'; 
    fileSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    Location = fscanf(fileID1,formatSpec, fileSize);
    fclose(fileID1);   

    %Transpose the array so that * matches the orientation of the data in the file.
   Location = Location'


%% Plot data
	figure % new figure
    plot3(XData,Location(2),YData,Location(3), ZData,Location(4))
    title('Plotted Trajctory')
    ylabel('y-axis')
	xlabel('x-axis')
    zlabel('z-axis')
    
    figure
    plot(Location(1),Location(5))
    title('Plotted Trajctory-PHI')
    ylabel('PHI')
	xlabel('time (s)')
    
    %%
	ax1 = subplot(3,1,1); % top subplot
	ax2 = subplot(3,1,2); % middle subplot
	ax3 = subplot(3,1,3); % bottom subplot

	plot(ax1,joint1(:,1),joint1(:,2),'-*k')
	title(ax1,strcat('Theta ','1'))

	ylabel(ax1,strcat('Theta ','1'))
	xlabel(ax1,'time(UNITS??)' )

	plot(ax2,joint1(:,1),joint1(:,3),'-*k')
	ylabel(ax2,'Velocity ')
	xlabel(ax2,'time(UNITS??)' )

	plot(ax3,joint1(:,1),joint1(:,4),'-*k')
	ylabel(ax3,'Accelaration')
	xlabel(ax3,'time(UNITS??)' )






