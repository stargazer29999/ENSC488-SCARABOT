function T=inverseT(T)

t=T(1:3, 1:3);
t=transpose(t);

tp=-t*T(1:3, 4);

T(1:3, 1:3)=t;
T(1:3, 4)=tp;

end
