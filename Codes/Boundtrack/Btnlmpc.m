clc;close all;clear;

r=20;
x=5;
y=7;

th=0:pi/50:2*pi;

for i=1:size(th,2)
  
xp(i) = r * cos(th(i)) + x;
yp(i) = r * sin(th(i)) + y;
end

mv1=[0];
mv2=[0];
mv3=[0];
mv4=[0];
x1=[0];
x2=[0];
x3=[0];
x4=[0];
x5=[0];
x6=[0];

for k=1:size(xp,2)-1
Xs=[xp(k) yp(k) th(k) 0 0 0];
Xf=[xp(k+1) yp(k+1) th(k+1) 0 0 0];
Infor=nonlimpc(Xs',Xf,3);
mv1=[mv1;Infor.MVopt(:,1)];
mv2=[mv2;Infor.MVopt(:,2)];
mv3=[mv3;Infor.MVopt(:,3)];
mv4=[mv4;Infor.MVopt(:,4)];
x1=[x1;Infor.Xopt(:,1)];
x2=[x2;Infor.Xopt(:,2)];
x3=[x3;Infor.Xopt(:,3)];
x4=[x4;Infor.Xopt(:,4)];
x5=[x5;Infor.Xopt(:,5)];
x6=[x6;Infor.Xopt(:,6)];
plot(Infor.Xopt(:,1),Infor.Xopt(:,2),'ko','MarkerSize',10)
hold on;
end
hold off

ts=0:0.4:0.4*4*(size(xp,2)-1);
figure()
plot(ts,mv1)

figure()
plot(ts,mv2)

figure()
plot(ts,mv3)

figure()
plot(ts,mv4)

figure()
plot(ts,x1)

figure()
plot(ts,x2)

figure()
plot(ts,x3)

figure()
plot(ts,x4)

figure()
plot(ts,x5)

figure()
plot(ts,x6)



function info=nonlimpc(Xini,Xfin,pr)

%no of state,inputs and outputs for non linear mpc controller
nx=6; %no of prediction model states
ny=6; %no of prediction model outputs
nu=4; %no of prediction model inputs

%create a non linear object whose prediction model has nx states, ny outputs, and nu inputs, where all inputs are manipulated variables.
nlobj=nlmpc(nx,ny,nu);

%prediction model Sample time
Ts=0.4;
nlobj.Ts = Ts;

%prediction horizon steps
p=pr;
nlobj.PredictionHorizon = p;

%control horizon steps
c=5;
nlobj.ControlHorizon = c;

X_end=Xfin;

nlobj.Model.StateFcn = @FlyingRobotStateFcn;
nlobj.Jacobian.StateFcn = @FlyingRobotStateJacobianFcn;
% %Optimisation function based on minimisation of thrust inputs thereby fuel
nlobj.Optimization.CustomCostFcn = @(X,U,e,data) Ts*sum(sum(U(1:p,:)));
nlobj.Optimization.ReplaceStandardCost = true;

nlobj.Optimization.CustomEqConFcn = @(X,U,data) X(end,:)'-X_end';

% min_thrust = 0; % Minimum thrust constraint
% max_thrust = 1;

for ct = 1:nu
    nlobj.MV(ct).Min =0;% min_thrust * ones(p,1);
    nlobj.MV(ct).Max =1; %max_thrust * ones(p,1);
end
 

x0 = Xini;
u0 = zeros(nu,1);
validateFcns(nlobj,x0,u0);

[~,~,info] = nlmpcmove(nlobj,x0,u0);

% FlyingRobotPlotPlanning(info,Ts)

end

function FlyingRobotPlotPlanning(Info,Ts)
Xopt = Info.Xopt;
MVopt = Info.MVopt;
fprintf('Optimal fuel consumption = %10.6f\n',Info.Cost*Ts)
t = Info.Topt;
figure;
states = {'x','y','theta','vx','vy','omega'};
for i = 1:size(Xopt,2)
    subplot(3,2,i)
    plot(t,Xopt(:,i),'o-')
    title(states{i})
end
figure;
MVopt(end,:) = 0; % replace the last row u(k+p) with 0
for i = 1:4
    subplot(4,1,i)
    stairs(t,MVopt(:,i),'o-')
    axis([0 max(t) -0.1 1.1])
    title(sprintf('Thrust u(%i)', i));
end
figure;
hold off
for ct=1:size(Xopt,1)
    lf = [cos(atan(0.5)+Xopt(ct,3))*0.5 sin(atan(0.5)+Xopt(ct,3))*0.5];
    rf = [cos(atan(-0.5)+Xopt(ct,3))*0.5 sin(atan(-0.5)+Xopt(ct,3))*0.5];
    lr = [cos(pi-atan(0.5)+Xopt(ct,3))*0.5 sin(pi-atan(0.5)+Xopt(ct,3))*0.5];
    rr = [cos(pi-atan(-0.5)+Xopt(ct,3))*0.5 sin(pi-atan(-0.5)+Xopt(ct,3))*0.5];
    patch([lf(1) rf(1) rr(1) lr(1)]+Xopt(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xopt(ct,2),'y','FaceAlpha',0.5,'LineStyle',':');
    line([lf(1) rf(1)]+Xopt(ct,1),[lf(2) rf(2)]+Xopt(ct,2),'color','r');
    hold on
end
xlabel('x')
ylabel('y')
title('Optimal Trajectory')
end



function dxdt=FlyingRobotStateFcn(x,u)
% Parameters
alpha = 0.2;
beta  = 0.2;

% Variables
phi = x(3);
T1 = u(1)- u(2);
T2 = u(3)- u(4);

% Wind velocity components
wind_x = 1;
wind_y = 1;


% State equations
dxdt = zeros(6, 1);

% Linear velocities
dxdt(1) = x(4) + wind_x; % x velocity
dxdt(2) = x(5) + wind_y; % y velocity
dxdt(3) = x(6);
dxdt(4) = (T1 + T2)*cos(phi);
dxdt(5) = (T1 + T2)*sin(phi);
dxdt(6) = alpha*T1 - beta*T2;
end

function [A, B] = FlyingRobotStateJacobianFcn(x, u)
% Parameters
alpha = 0.2;
beta  = 0.2;

% Variables
phi = x(3);
T1 = u(1)-u(2);
T2 = u(3)-u(4);

% Linearize the state equations at the current condition
A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,3) = -(T1 + T2)*sin(phi);
A(5,3) =  (T1 + T2)*cos(phi);
B = zeros(6,4);
B(4,:) = cos(phi)*[1 -1 1 -1];
B(5,:) = sin(phi)*[1 -1 1 -1];
B(6,:) = [alpha*[1 -1], -beta*[1 -1]];
end




