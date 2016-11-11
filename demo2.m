%Get Matlab to read text file
%output: graphs of each joint
% Matlab must point to the correct file location

%Read Files
joint1 = readFile('joint1.txt')
joint2 = readFile('joint2.txt')
joint3 = readFile('joint3.txt')
joint4 = readFile('joint4.txt')

plotJoints(joint1, '1')
plotJoints(joint2, '2')
plotJoints(joint3, '3')
plotJoints(joint4, '4')


