clc;clear;close all;

% % % prompt = "Enter the Coordinates of ignition point as a 1x2 matrix with []-";
% % C = input(prompt);
% %
% % prompt3 = "Enter the Grid size of the environment";
% % Ng = input(prompt3);
% %
% % fprintf('x(1)-x inertial coordinate of COM \nx(2)-y inertial coordinate of COM \nx(3)-theta-thrust direction \nx(4)-vx-velocity of x \nx(5)-vy-velocity of y \nx(6)-omega-angular velocity of theta \n')
% % X0 = input('Enter the Initial position and Orientation of the UAV ');

C=[23 21];
Ng=40;
lst=[];
xig=C(1);
yig=C(2);
t0=30;

if C(1)<=Ng/2
    Quadone(xig,yig,Ng,t0,lst);
    
elseif C(1)>Ng/2
    Quadtwo(xig,yig,Ng,t0,lst);
    
end

