clc;clear;close all;

% prompt = "Enter the Size of the Field in m2";
% Ng = input(prompt);
% 
% fprintf('x(1)-x inertial coordinate of COM \nx(2)-y inertial coordinate of COM \nx(3)-theta-thrust direction \nx(4)-vx-velocity of x \nx(5)-vy-velocity of y \nx(6)-omega-angular velocity of theta \n')
% X0 = input('Enter the Initial position and Orientation of the UAV ');
% 
% d = input('Enter the coverage area diameter of the sprayer in UAV in m');

Ng=24;
X0=[Ng;Ng;0;0;0;0];
d=1;

r=d/2;

X1=[Ng-r Ng-r pi/2 0 0 0];

Infor=nonlimpc(X0,X1,15);
Ttot=Infor.Topt;
u1to=Infor.MVopt(:,1);
u2to=Infor.MVopt(:,2);
u3to=Infor.MVopt(:,3);
u4to=Infor.MVopt(:,4);

Xpa=Infor.Xopt(:,1);
Ypa=Infor.Xopt(:,2);

for n=1:4:2*Ng

X2=[Ng-r Ng-n*r 0 0 0 0];
Infor2=nonlimpc(X1',X2,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor2.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor2.Topt];
u1to=[u1to;Infor2.MVopt(:,1)];
u2to=[u2to;Infor2.MVopt(:,2)];
u3to=[u3to;Infor2.MVopt(:,3)];
u4to=[u4to;Infor2.MVopt(:,4)];

Xpa=[Xpa;Infor2.Xopt(:,1)];
Ypa=[Ypa;Infor2.Xopt(:,2)];

X3=[r Ng-n*r 0 0 0 0];
Infor3=nonlimpc(X2',X3,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor3.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor3.Topt];
u1to=[u1to;Infor3.MVopt(:,1)];
u2to=[u2to;Infor3.MVopt(:,2)];
u3to=[u3to;Infor3.MVopt(:,3)];
u4to=[u4to;Infor3.MVopt(:,4)];

Xpa=[Xpa;Infor3.Xopt(:,1)];
Ypa=[Ypa;Infor3.Xopt(:,2)]; 

X4=[r Ng-n*r pi/2 0 0 0];
Infor4=nonlimpc(X3',X4,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor4.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor4.Topt];
u1to=[u1to;Infor4.MVopt(:,1)];
u2to=[u2to;Infor4.MVopt(:,2)];
u3to=[u3to;Infor4.MVopt(:,3)];
u4to=[u4to;Infor4.MVopt(:,4)];

Xpa=[Xpa;Infor4.Xopt(:,1)];
Ypa=[Ypa;Infor4.Xopt(:,2)]; 

X5=[r Ng-((n+2)*r) pi/2 0 0 0];
Infor5=nonlimpc(X4',X5,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor5.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor5.Topt];
u1to=[u1to;Infor5.MVopt(:,1)];
u2to=[u2to;Infor5.MVopt(:,2)];
u3to=[u3to;Infor5.MVopt(:,3)];
u4to=[u4to;Infor5.MVopt(:,4)];

Xpa=[Xpa;Infor5.Xopt(:,1)];
Ypa=[Ypa;Infor5.Xopt(:,2)]; 

X6=[r Ng-((n+2)*r) pi 0 0 0];
Infor6=nonlimpc(X5',X6,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor6.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor6.Topt];
u1to=[u1to;Infor6.MVopt(:,1)];
u2to=[u2to;Infor6.MVopt(:,2)];
u3to=[u3to;Infor6.MVopt(:,3)];
u4to=[u4to;Infor6.MVopt(:,4)];
Xpa=[Xpa;Infor6.Xopt(:,1)];
Ypa=[Ypa;Infor6.Xopt(:,2)]; 

X7=[Ng-r Ng-((n+2)*r) pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor7.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor7.Topt];
u1to=[u1to;Infor7.MVopt(:,1)];
u2to=[u2to;Infor7.MVopt(:,2)];
u3to=[u3to;Infor7.MVopt(:,3)];
u4to=[u4to;Infor7.MVopt(:,4)];

Xpa=[Xpa;Infor7.Xopt(:,1)];
Ypa=[Ypa;Infor7.Xopt(:,2)]; 

X8=[Ng-r Ng-((n+2)*r) pi/2 0 0 0];
Infor8=nonlimpc(X7',X8,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor8.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor8.Topt];
u1to=[u1to;Infor8.MVopt(:,1)];
u2to=[u2to;Infor8.MVopt(:,2)];
u3to=[u3to;Infor8.MVopt(:,3)];
u4to=[u4to;Infor8.MVopt(:,4)];

Xpa=[Xpa;Infor8.Xopt(:,1)];
Ypa=[Ypa;Infor8.Xopt(:,2)];

X9=[Ng-r Ng-((n+4)*r) pi/2 0 0 0];
Infor9=nonlimpc(X8',X9,5);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor9.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor9.Topt];
u1to=[u1to;Infor9.MVopt(:,1)];
u2to=[u2to;Infor9.MVopt(:,2)];
u3to=[u3to;Infor9.MVopt(:,3)];
u4to=[u4to;Infor9.MVopt(:,4)];

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
xlim([0 24])
ylim([0 24])

figure();
plot(Ttot,u1to);

figure();
plot(Ttot,u2to);

figure();
plot(Ttot,u3to);

figure();
plot(Ttot,u4to);


% figure()
% for i=1:size(Xpa)
% th = 0:pi/50:2*pi;
% xunit = 0.5 * cos(th) + Xpa(i);
% yunit = 0.5 * sin(th) + Ypa(i);
% h = plot(xunit, yunit);
% hold on;
% end
% hold off;
