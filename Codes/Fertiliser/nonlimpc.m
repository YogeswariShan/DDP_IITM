function info=nonlimpc(Xini,Xfin,pr)

%no of state,inputs and outputs for non linear mpc controller
nx=6; %no of prediction model states
ny=6; %no of prediction model outputs
nu=4; %no of prediction model inputs

%create a non linear object whose prediction model has nx states, ny outputs, and nu inputs, where all inputs are manipulated variables.
nlobj=nlmpc(nx,ny,nu);

%prediction model Sample time
Ts=0.35;
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



