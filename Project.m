%% Forward Kinematics
syms c1 s1 %a1 d1
a1=195
d1=70

syms c2 s2 %d2 alf2 
alf2=pi
a2=142

syms c3 s3 d3 d3
D3=140+270+d3

syms c4 s4 %a4 d4
d4=140

syms  c5 s5  %a5 d5

syms theta1 theta4 theta2
disp('***units are radians and mm***')

%% Intital T matrices
Ta=[c1 -s1 0 0; 
    s1 c1 0 0; 
    0 0 1 d1; 
    0 0 0 1];
%t=matrixT(0,0,theta1, d1)

Tb=[c2 -s2 0 a1;
    s2 c2 0 0;
    0 0 1 0; 
    0 0 0 1];
%t=matrixT(0,a1,theta2, 0)

Tc=[cos(alf2) 0 0 a2;
    0 cos(alf2) 0 0; 
    0 0 -1 cos(alf2)*D3; 
    0 0 0 1];
%t=matrixT(alf2,a2,0, d3)

Td=[c4 -s4 0 0;
    s4 c4 0 0; 
    0 0 1 d4; 
    0 0 0 1];
%t=matrixT(0,0,theta4, d4)

Te=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Ta
Tb
%get from frame 0 to frame 2
Tab=Ta*Tb 
disp('*** Using: sin(a + b) = sin(a)cos(b) + cos(a)sin(b) ***')
disp('*** cos(a + b) = cos(a)cos(b) – sin(a)sin(b) ***')

syms c12 s12
Tab(1,1)=c12;
Tab(1,2)=-s12;
Tab(2,1)=s12;
Tab(2,2)=c12;

Tab
Tc
%get from frame 0 to frame 3
Tac=Tab*Tc

%get from frame 0 to frame 4
Td
Tad=Tac*Td
syms s124
disp('*** same trigometric identities as before ***')
Tad(2,1)=-s124;
Tad(1,2)=s124;
Tad

% %get from frame 0 to frame 5
Te
Tae=Tad*Te
P=Tae(:,4)
R=Tae(1:3,1:3)

%% Inverse Kinematics
syms px py pz
syms r11 r12 r13 r21 r22 r23 r31 r32 r33
T=[r11 r12 r13 px; r21 r22 r23 py; r31 r32 r33 pz; 0 0 0 1]
display('*****Find value of d3*****')
d3_=-Tae(3,4)-480

display('*****Find value of Theta1*****')

TaINV_T=inverseT(Ta)*T
Tbe=Tb*Tc*Td*Te

disp('*from elems(2,4) of TaINV_T and Tbe*****')
TaINV_T(2,3)==0
tan(theta1)==r13/r23
%%using lecture 10+11 identity (4)
theta1=atan2(r13/r23,1)
theta1_=atan2(-r13/r23,-1)+pi()
disp('theta1**=theta1+pi()')


display('*****Find value of Theta2*****')
TaINV_T(1,4)==Tbe(1,4)
c2=(TaINV_T(1,4)-195)/142
%%using lecture 10+11 identity (2)
theta2=atan2(sqrt(1-c2^2),c2)
theta2_=atan2(-sqrt(1-c2^2),c2)
disp('apply for both theta1 and theta1** ')



display('*****Find value of Theta4*****')
c=TaINV_T (1,2)
a=s2
b=c2
%%using lecture 10+11 identity (5)
if (c+a)==0
    disp('*Degenerate case*')
    theta4=2*atan2(-a/b,1)
   disp('apply for both theta2 and theta2** for a total of 2 cases')

elseif b==0
    disp('*Degenerate case*')
    if c==0     
        disp('*infinite solutions*')
    else
        disp('no solution')
    end
else
    theta4=atan2((b+sqrt(b^2-c^2+a^2))/(c+a),1)
    theta4_=atan2((b-sqrt(b^2-c^2+a^2))/(c+a),1)
    disp('apply for both theta2 and theta2** for a total of four cases')
end

