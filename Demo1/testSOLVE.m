function joints=testSOLVE(Jold,Tmatrix)
    %Find value of d3
	joints(3) = -Tmatrix(3, 4) - 480;

	%Find value of Theta1
    if(Tmatrix(1, 3)==0 && Tmatrix(2, 3)==0)
        theta1= atan2(0,1)
        theta1_=atan2(0,1)+pi()
    else
        theta1 = atan2(Tmatrix(1, 3)/ Tmatrix(2, 3), 1)
        theta1_ = atan2(-Tmatrix(1, 3) / Tmatrix(2, 3), -1) + pi()
    end
    
	%Find value of Theta2
    C2=cos(theta1)*Tmatrix(1,4)/142+ Tmatrix(2, 4)*sin(theta1)/142-195/142
    C=cast(C2,'int16');
    if(C==1)
         theta2 = atan2(0,1)
         theta2_= atan2(0,1)+pi()
   elseif (C^2>1)
        theta2_=NaN
        theta2_=NaN
    else
        theta2 = atan2((1-C2^2)^(.5),C2)
        theta2_= atan2(-(1-C2^2)^(.5),C2)
    end
    
    C2=cos(theta1_)*Tmatrix(1,4)/142+ Tmatrix(2, 4)*sin(theta1_)/142-195/142
    C=cast(C2,'int16');
    if(C==1)
         theta2_1= atan2(0,1)
         theta2_2= atan2(0,1)+pi()
    elseif(C^2>1)
        theta2_1=NaN
        theta2_2=NaN
    else
        theta2_1= atan2((1-C2^2)^(.5),C2)
        theta2_2= atan2(-(1-C2^2)^(.5),C2)
   end
    
	%Find value of Theta4
	theta4=findTheta4(Tmatrix,theta1,theta2)
    theta4_=findTheta4(Tmatrix,theta1,theta2_)
    theta4_1=findTheta4(Tmatrix,theta1_,theta2_1)
    theta4_2=findTheta4(Tmatrix,theta1,theta2_2)
    
    %find invalid cases
    
    
    %find shortest distance
    theta1=theta1*180/pi();
    theta1_=theta1_*180/pi();
    theta2=theta2*180/pi();
    theta2_=theta2_*180/pi();
    theta2_1=theta2_1*180/pi();
    theta2_1=theta2_1*180/pi();
    theta2_2=theta2_2*180/pi();
    theta4=theta4*180/pi();
    theta4_=theta4_*180/pi();
    theta4_1=theta4_1*180/pi();
    theta4_2=theta4_2*180/pi();
    
    P=zeros(8,1)
    P(1)=abs(theta1-Jold(1))+abs(theta2-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4(1)-Jold(4))
    P(2)=abs(theta1-Jold(1))+abs(theta2-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4(2)-Jold(4))
    P(3)=abs(theta1-Jold(1))+abs(theta2_-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_(1)-Jold(4))
    P(4)=abs(theta1-Jold(1))+abs(theta2_-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_(2)-Jold(4))
    P(5)=abs(theta1_-Jold(1))+abs(theta2_1-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_1(1)-Jold(4))
    P(6)=abs(theta1_-Jold(1))+abs(theta2_1-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_1(2)-Jold(4))   
    P(7)=abs(theta1_-Jold(1))+abs(theta2_2-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_2(1)-Jold(4))
    P(8)=abs(theta1_-Jold(1))+abs(theta2_2-Jold(2))+abs(joints(3)-Jold(3))+abs(theta4_2(2)-Jold(4))
    
    [minColVal, minColIdx] = min(P);
    %save to output
   %joints(1)=
   % joints(2)=
   % joints(4)=
    
    
    end 
