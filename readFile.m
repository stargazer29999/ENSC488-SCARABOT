%http://www.mathworks.com/help/matlab/ref/fscanf.html
function joint = readFile(fileNAME)

    %open the file for reading, and obtain the file identifier, fileID.
    fileID = fopen(fileNAME,'r');

    %Define the format of the data to read and the shape of the output array.
    formatSpec = '%f %f %f %f'; 
    jointSize = [4 Inf];

    %Read the file data, filling output array, *, in column order. 
    %fscanf reuses the format, formatSpec, throughout the file.
    joint = fscanf(fileID,formatSpec, jointSize);
    fclose(fileID);

    %Transpose the array so that * matches the orientation of the data in the file.
    joint = joint';

end