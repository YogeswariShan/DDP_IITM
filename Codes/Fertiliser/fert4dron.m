clc;clear;close all;

% prompt = "Enter the Size of the Field in m2";
% Ng = input(prompt);
% 
% fprintf('x(1)-x inertial coordinate of COM \nx(2)-y inertial coordinate of COM \nx(3)-theta-thrust direction \nx(4)-vx-velocity of x \nx(5)-vy-velocity of y \nx(6)-omega-angular velocity of theta \n')
% X0 = input('Enter the Initial position and Orientation of the UAV ');
% 
% d = input('Enter the coverage area diameter of the sprayer in UAV in m');

Ng=24;
X0=[Ng;Ng;(3/4)*pi;0;0;0];
d=1;

r=d/2;

X1=[Ng-r Ng-r pi/2 0 0 0];

Infor=nonlimpc(X0,X1,15);

Xpa=Infor.Xopt(:,1);
Ypa=Infor.Xopt(:,2);

for n=1:4:(1/2)*Ng

X2=[Ng-r Ng-n*r 0 0 0 0];
Infor2=nonlimpc(X1',X2,5);

Xpa=[Xpa;Infor2.Xopt(:,1)];
Ypa=[Ypa;Infor2.Xopt(:,2)];

X3=[r Ng-n*r 0 0 0 0];
Infor3=nonlimpc(X2',X3,5);

Xpa=[Xpa;Infor3.Xopt(:,1)];
Ypa=[Ypa;Infor3.Xopt(:,2)]; 

X4=[r Ng-n*r pi/2 0 0 0];
Infor4=nonlimpc(X3',X4,5);

Xpa=[Xpa;Infor4.Xopt(:,1)];
Ypa=[Ypa;Infor4.Xopt(:,2)]; 

X5=[r Ng-((n+2)*r) pi/2 0 0 0];
Infor5=nonlimpc(X4',X5,5);

Xpa=[Xpa;Infor5.Xopt(:,1)];
Ypa=[Ypa;Infor5.Xopt(:,2)]; 

X6=[r Ng-((n+2)*r) pi 0 0 0];
Infor6=nonlimpc(X5',X6,5);

Xpa=[Xpa;Infor6.Xopt(:,1)];
Ypa=[Ypa;Infor6.Xopt(:,2)]; 

X7=[Ng-r Ng-((n+2)*r) pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);

Xpa=[Xpa;Infor7.Xopt(:,1)];
Ypa=[Ypa;Infor7.Xopt(:,2)]; 

X8=[Ng-r Ng-((n+2)*r) pi/2 0 0 0];
Infor8=nonlimpc(X7',X8,5);

Xpa=[Xpa;Infor8.Xopt(:,1)];
Ypa=[Ypa;Infor8.Xopt(:,2)];

X9=[Ng-r Ng-((n+4)*r) pi/2 0 0 0];
Infor9=nonlimpc(X8',X9,5);

Xpa=[Xpa;Infor9.Xopt(:,1)];
Ypa=[Ypa;Infor9.Xopt(:,2)];

X1=X9;
end

% X10=[Ng-r Ng-(2*(Ng-1)+1)*r 0 0 0 0];
% Infor10=nonlimpc(X9',X10,5);
% 
% Xpa=[Xpa;Infor10.Xopt(:,1)];
% Ypa=[Ypa;Infor10.Xopt(:,2)];
% 
% X11=[r Ng-(2*(Ng-1)+1)*r 0 0 0 0];
% Infor11=nonlimpc(X10',X11,7);
% 
% Xpa=[Xpa;Infor11.Xopt(:,1)];
% Ypa=[Ypa;Infor11.Xopt(:,2)];


figure()
plot(Xpa,Ypa,'-')
hold on;

X1=[Ng-r Ng-((1/2)*Ng*r) pi/2 0 0 0];

Infor=nonlimpc(X0,X1,15);

Xpa=Infor.Xopt(:,1);
Ypa=Infor.Xopt(:,2);

for n=((1/2)*Ng)+1:4:Ng

X2=[Ng-r Ng-n*r 0 0 0 0];
Infor2=nonlimpc(X1',X2,5);

Xpa=[Xpa;Infor2.Xopt(:,1)];
Ypa=[Ypa;Infor2.Xopt(:,2)];

X3=[r Ng-n*r 0 0 0 0];
Infor3=nonlimpc(X2',X3,5);

Xpa=[Xpa;Infor3.Xopt(:,1)];
Ypa=[Ypa;Infor3.Xopt(:,2)]; 

X4=[r Ng-n*r pi/2 0 0 0];
Infor4=nonlimpc(X3',X4,5);

Xpa=[Xpa;Infor4.Xopt(:,1)];
Ypa=[Ypa;Infor4.Xopt(:,2)]; 

X5=[r Ng-((n+2)*r) pi/2 0 0 0];
Infor5=nonlimpc(X4',X5,5);

Xpa=[Xpa;Infor5.Xopt(:,1)];
Ypa=[Ypa;Infor5.Xopt(:,2)]; 

X6=[r Ng-((n+2)*r) pi 0 0 0];
Infor6=nonlimpc(X5',X6,5);

Xpa=[Xpa;Infor6.Xopt(:,1)];
Ypa=[Ypa;Infor6.Xopt(:,2)]; 

X7=[Ng-r Ng-((n+2)*r) pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);

Xpa=[Xpa;Infor7.Xopt(:,1)];
Ypa=[Ypa;Infor7.Xopt(:,2)]; 

X8=[Ng-r Ng-((n+2)*r) pi/2 0 0 0];
Infor8=nonlimpc(X7',X8,5);

Xpa=[Xpa;Infor8.Xopt(:,1)];
Ypa=[Ypa;Infor8.Xopt(:,2)];

X9=[Ng-r Ng-((n+4)*r) pi/2 0 0 0];
Infor9=nonlimpc(X8',X9,5);

Xpa=[Xpa;Infor9.Xopt(:,1)];
Ypa=[Ypa;Infor9.Xopt(:,2)];

X1=X9;
end


plot(Xpa,Ypa,'-')
hold on;

X1=[Ng-r Ng-(Ng*r) pi/2 0 0 0];

Infor=nonlimpc(X0,X1,15);

Xpa=Infor.Xopt(:,1);
Ypa=Infor.Xopt(:,2);

for n=(Ng)+1:4:(3/2)*Ng

X2=[Ng-r Ng-n*r 0 0 0 0];
Infor2=nonlimpc(X1',X2,5);

Xpa=[Xpa;Infor2.Xopt(:,1)];
Ypa=[Ypa;Infor2.Xopt(:,2)];

X3=[r Ng-n*r 0 0 0 0];
Infor3=nonlimpc(X2',X3,5);

Xpa=[Xpa;Infor3.Xopt(:,1)];
Ypa=[Ypa;Infor3.Xopt(:,2)]; 

X4=[r Ng-n*r pi/2 0 0 0];
Infor4=nonlimpc(X3',X4,5);

Xpa=[Xpa;Infor4.Xopt(:,1)];
Ypa=[Ypa;Infor4.Xopt(:,2)]; 

X5=[r Ng-((n+2)*r) pi/2 0 0 0];
Infor5=nonlimpc(X4',X5,5);

Xpa=[Xpa;Infor5.Xopt(:,1)];
Ypa=[Ypa;Infor5.Xopt(:,2)]; 

X6=[r Ng-((n+2)*r) pi 0 0 0];
Infor6=nonlimpc(X5',X6,5);

Xpa=[Xpa;Infor6.Xopt(:,1)];
Ypa=[Ypa;Infor6.Xopt(:,2)]; 

X7=[Ng-r Ng-((n+2)*r) pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);

Xpa=[Xpa;Infor7.Xopt(:,1)];
Ypa=[Ypa;Infor7.Xopt(:,2)]; 

X8=[Ng-r Ng-((n+2)*r) pi/2 0 0 0];
Infor8=nonlimpc(X7',X8,5);

Xpa=[Xpa;Infor8.Xopt(:,1)];
Ypa=[Ypa;Infor8.Xopt(:,2)];

X9=[Ng-r Ng-((n+4)*r) pi/2 0 0 0];
Infor9=nonlimpc(X8',X9,5);

Xpa=[Xpa;Infor9.Xopt(:,1)];
Ypa=[Ypa;Infor9.Xopt(:,2)];

X1=X9;
end


plot(Xpa,Ypa,'-')
hold on;

X1=[Ng-r Ng-((3/2)*Ng*r) pi/2 0 0 0];

Infor=nonlimpc(X0,X1,15);

Xpa=Infor.Xopt(:,1);
Ypa=Infor.Xopt(:,2);

for n=((3/2)*Ng)+1:4:2*Ng

X2=[Ng-r Ng-n*r 0 0 0 0];
Infor2=nonlimpc(X1',X2,5);

Xpa=[Xpa;Infor2.Xopt(:,1)];
Ypa=[Ypa;Infor2.Xopt(:,2)];

X3=[r Ng-n*r 0 0 0 0];
Infor3=nonlimpc(X2',X3,5);

Xpa=[Xpa;Infor3.Xopt(:,1)];
Ypa=[Ypa;Infor3.Xopt(:,2)]; 

X4=[r Ng-n*r pi/2 0 0 0];
Infor4=nonlimpc(X3',X4,5);

Xpa=[Xpa;Infor4.Xopt(:,1)];
Ypa=[Ypa;Infor4.Xopt(:,2)]; 

X5=[r Ng-((n+2)*r) pi/2 0 0 0];
Infor5=nonlimpc(X4',X5,5);

Xpa=[Xpa;Infor5.Xopt(:,1)];
Ypa=[Ypa;Infor5.Xopt(:,2)]; 

X6=[r Ng-((n+2)*r) pi 0 0 0];
Infor6=nonlimpc(X5',X6,5);

Xpa=[Xpa;Infor6.Xopt(:,1)];
Ypa=[Ypa;Infor6.Xopt(:,2)]; 

X7=[Ng-r Ng-((n+2)*r) pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);

Xpa=[Xpa;Infor7.Xopt(:,1)];
Ypa=[Ypa;Infor7.Xopt(:,2)]; 

X8=[Ng-r Ng-((n+2)*r) pi/2 0 0 0];
Infor8=nonlimpc(X7',X8,5);

Xpa=[Xpa;Infor8.Xopt(:,1)];
Ypa=[Ypa;Infor8.Xopt(:,2)];

X9=[Ng-r Ng-((n+4)*r) pi/2 0 0 0];
Infor9=nonlimpc(X8',X9,5);

Xpa=[Xpa;Infor9.Xopt(:,1)];
Ypa=[Ypa;Infor9.Xopt(:,2)];

X1=X9;
end

plot(Xpa,Ypa,'-')
hold off;

% figure()
% for i=1:size(Xpa)
% th = 0:pi/50:2*pi;
% xunit = 0.5 * cos(th) + Xpa(i);
% yunit = 0.5 * sin(th) + Ypa(i);
% h = plot(xunit, yunit);
% hold on;
% end
% hold off;
