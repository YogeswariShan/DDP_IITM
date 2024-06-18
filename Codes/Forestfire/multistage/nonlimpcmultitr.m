function info=nonlimpcmultitr(Xini,Xfin,pr)

Ts = 0.5;
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
FlyingRobotPlotPlanning(info,Ts);

ny = 6;
nlobj_tracking = nlmpc(nx,ny,nu);
nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Model.StateJacFcn;
nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 20;
nlobj_tracking.ControlHorizon = 10;
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);
for ct = 1:nu
    nlobj_tracking.MV(ct).Min = 0;
    nlobj_tracking.MV(ct).Max = 1;
end
nlobj_tracking.Optimization.CustomEqConFcn = ...
    @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

validateFcns(nlobj_tracking,x0,u0);
DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);
DMeasFcn = @(xk) xk(1:3);
EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
EKF.MeasurementNoise = 0.01;
Tsteps = pr;
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);
Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:);repmat(Xopt(end,:),Tsteps-p,1)];

hbar = waitbar(0,'Simulation Progress');
options = nlmpcmoveopt;

for k = 1:Tsteps

    % Obtain plant output measurements with sensor noise.
    yk = xHistory(k,1:3)' + randn*0.01;

    % Correct state estimation based on the measurements.
    xk = correct(EKF, yk);

    % Compute the control moves with reference previewing.
    [uk,options] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);

    % Predict the state for the next step.
    predict(EKF,uk,Ts);

    % Store the control move and update the last MV for the next step.
    uHistory(k,:) = uk'; %#ok<*SAGROW>
    lastMV = uk;

    % Update the real plant states for the next step by solving the
    % continuous-time ODEs based on current states xk and input uk.
    ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');

    % Store the state values.
    xHistory(k+1,:) = YOUT(end,:);

    % Update the status bar.
    waitbar(k/Tsteps, hbar);
end
close(hbar)
FlyingRobotPlotTracking(info,Ts,p,Tsteps,xHistory,uHistory);


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

function J=FlyingRobotCostFcn(stage,x,u)
J = u(1) + u(2) + u(3) + u(4);
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
wind_x = 0;
wind_y = 0;


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



function FlyingRobotPlotTracking(info,Ts,Psteps,Tsteps,Xcl,Ucl)
% FlyingRobotPlotTracking displays the optimal trajectory of the
% flying robot.

% Copyright 2018-2021 The MathWorks, Inc.
Xopt = info.Xopt;
tp = Ts*(0:Psteps);
tt = Ts*(0:Tsteps);
figure;
states = {'x1','x2','theta','v1','v2','omega'};
for i = 1:6
    subplot(3,2,i)
    plot(tt,Xcl(:,i),'+',tp,Xopt(:,i),'-')
    legend('actual','plan','location','northwest')
    title(states{i})
end
figure;
for i = 1:4
    subplot(4,1,i)
    stairs(tt(1:end-1),Ucl(:,i))
    title(sprintf('Thrust u(%i)', i));
    axis([0 tt(end) -0.1 1.1])
    hold on
    stairs(tp(1:end-1),info.MVopt(1:end-1,i))
    legend('actual','plan')
    hold off
end
figure;
hold on
for ct=1:size(Xopt,1)
    lf = [cos(atan(0.5)+Xopt(ct,3))*0.5 sin(atan(0.5)+Xopt(ct,3))*0.5];
    rf = [cos(atan(-0.5)+Xopt(ct,3))*0.5 sin(atan(-0.5)+Xopt(ct,3))*0.5];
    lr = [cos(pi-atan(0.5)+Xopt(ct,3))*0.5 sin(pi-atan(0.5)+Xopt(ct,3))*0.5];
    rr = [cos(pi-atan(-0.5)+Xopt(ct,3))*0.5 sin(pi-atan(-0.5)+Xopt(ct,3))*0.5];
    patch([lf(1) rf(1) rr(1) lr(1)]+Xopt(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xopt(ct,2),'y','FaceAlpha',0.5,'LineStyle',':');
end
for ct=1:size(Xcl,1)
    lf = [cos(atan(0.5)+Xcl(ct,3))*0.5 sin(atan(0.5)+Xcl(ct,3))*0.5];
    rf = [cos(atan(-0.5)+Xcl(ct,3))*0.5 sin(atan(-0.5)+Xcl(ct,3))*0.5];
    lr = [cos(pi-atan(0.5)+Xcl(ct,3))*0.5 sin(pi-atan(0.5)+Xcl(ct,3))*0.5];
    rr = [cos(pi-atan(-0.5)+Xcl(ct,3))*0.5 sin(pi-atan(-0.5)+Xcl(ct,3))*0.5];
    if ct<size(Xcl,1)
        patch([lf(1) rf(1) rr(1) lr(1)]+Xcl(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xcl(ct,2),'b','FaceAlpha',0.5);
    else
        patch([lf(1) rf(1) rr(1) lr(1)]+Xcl(ct,1),[lf(2) rf(2) rr(2) lr(2)]+Xcl(ct,2),'r','FaceAlpha',0.5);
    end
end
xlabel('x')
ylabel('y')
title('Compare with Optimal Trajectory')
fprintf('Actual fuel consumption = %10.6f\n',sum(sum(Ucl(1:end-1,:)))*Ts);
end

function xk1 = FlyingRobotStateFcnDiscreteTime(xk, uk, Ts)
% Nonlinear discrete-time state transition for the flying robot model.
%
% Discretization uses the trapezoidal formula, assuming the input, uk, is
% constant during time interval Ts.
%
% The trapezoidal formula is implicit; that is, you cannot solve for xk1
% algebraically. You need to solve a set of nonlinear equations.  
% This function uses FSOLVE for this purpose.

% FlyingRobotStateFcn is the continuous-time state function for this
% example. Obtain the state derivatives at the current point.
xk = xk(:);     % Make sure inputs are column vectors
uk = uk(:);
ffun = @(xk,uk) FlyingRobotStateFcn(xk,uk);
fk = ffun(xk,uk);

% Extrapolation using xk1 = xk + Ts*fk is risky, since it might put xk1 in
% an infeasible region, which could prevent convergence. A safer
% alternative is xk1 = xk, but this method produces a poor estimate.
xk1 = xk + Ts*fk;

% Solve for xk1 satisfying the Trapezoidal rule.
FUN = @(xk1) TrapezoidalRule(xk,xk1,uk,Ts,fk,ffun);
Options = optimoptions('fsolve','Display','none');
xk1 = fsolve(FUN,xk1,Options);
end
% Trapezoidal rule function
function f = TrapezoidalRule(xk,xk1,uk,Ts,fk,ffun)
% Time derivatives at point xk1.
fk1 = ffun(xk1,uk);
% The following must be zero to satisfy the Trapezoidal Rule
f = xk1 - (xk + (Ts/2)*(fk1 + fk));

end



