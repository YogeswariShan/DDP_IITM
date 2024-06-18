function info=nonlimpcmulti(Xini,Xfin,pr)

Ts = 0.4;
p = pr;
nx = 6;
nu = 4;
nlobj = nlmpcMultistage(p,nx,nu);
nlobj.Ts = Ts;

nlobj.Model.StateFcn = @FlyingRobotStateFcn;
nlobj.Model.StateJacFcn = @FlyingRobotStateJacobianFcn;
for ct = 1:p
    nlobj.Stages(ct).CostFcn = @FlyingRobotCostFcn;
end

nlobj.Model.TerminalState=Xfin';

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

function J=FlyingRobotCostFcn(stage,x,u)
J = u(1) + u(2) + u(3) + u(4);
end



