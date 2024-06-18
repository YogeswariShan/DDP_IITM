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

function Quadone(xig,yig,Ng,t0,lst)


P=Environment(xig,yig,Ng,t0,lst);
X0=[Ng;0;0;0;0;0];
map = occupancyMap(P);
figure()
show(map)

% [maxValue, linearIndex] = max(P(:));
% [i0,j0] = ind2sub(size(P), linearIndex);
C0=[xig yig];
C1t=BottomSearch(C0,P);
C1=LeftSearch(C1t,P);
C1=[C1(1) C1(2)];
X1=[C1(1) C1(2) -pi 0 0 0];
Infor=nonlimpctr(X0,X1,20);
Ttot=Infor.Topt;
u1to=Infor.MVopt(:,1);
u2to=Infor.MVopt(:,2);
u3to=Infor.MVopt(:,3);
u4to=Infor.MVopt(:,4);
P=Environment(xig,yig,Ng,t0+Infor.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor.Xopt(:,1),Infor.Xopt(:,2),'ko','MarkerSize',10)

C2=RightSearch(C1,P);
X2=[C2(1) C2(2) -pi 0 0 0];
Infor2=nonlimpctr(X1',X2,20);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor2.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor2.Topt];
u1to=[u1to;Infor2.MVopt(:,1)];
u2to=[u2to;Infor2.MVopt(:,2)];
u3to=[u3to;Infor2.MVopt(:,3)];
u4to=[u4to;Infor2.MVopt(:,4)];

lst=[lst;getpathcells(C2,C1,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor2.Xopt(:,1),Infor2.Xopt(:,2),'ko','MarkerSize',10)

X20=[X2(1) X2(2) pi/2 0 0 0];
Infora=nonlimpctr(X2',X20,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora.Topt];
u1to=[u1to;Infora.MVopt(:,1)];
u2to=[u2to;Infora.MVopt(:,2)];
u3to=[u3to;Infora.MVopt(:,3)];
u4to=[u4to;Infora.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end),lst);

C3=TopSearch(C2,P);
X3=[C3(1) C3(2) pi/2 0 0 0];
Infor3=nonlimpctr(X20',X3,20);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor3.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor3.Topt];
u1to=[u1to;Infor3.MVopt(:,1)];
u2to=[u2to;Infor3.MVopt(:,2)];
u3to=[u3to;Infor3.MVopt(:,3)];
u4to=[u4to;Infor3.MVopt(:,4)];
lst=[lst;getpathcells(C3,C2,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infor3.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor3.Xopt(:,1),Infor3.Xopt(:,2),'ko','MarkerSize',10)

X30=[X3(1) X3(2) 0 0 0 0];
Infora1=nonlimpctr(X3',X30,20);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora1.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora1.Topt];
u1to=[u1to;Infora1.MVopt(:,1)];
u2to=[u2to;Infora1.MVopt(:,2)];
u3to=[u3to;Infora1.MVopt(:,3)];
u4to=[u4to;Infora1.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end),lst);

C4=LeftSearch(C3,P);
X4=[C4(1) C4(2) 0 0 0 0];
Infor4=nonlimpctr(X30',X4,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor4.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor4.Topt];
u1to=[u1to;Infor4.MVopt(:,1)];
u2to=[u2to;Infor4.MVopt(:,2)];
u3to=[u3to;Infor4.MVopt(:,3)];
u4to=[u4to;Infor4.MVopt(:,4)];
lst=[lst;getpathcells(C4,C3,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor4.Xopt(:,1),Infor4.Xopt(:,2),'ko','MarkerSize',10)

X40=[X4(1) X4(2) -pi/2 0 0 0];
Infora2=nonlimpctr(X4',X40,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora2.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora2.Topt];
u1to=[u1to;Infora2.MVopt(:,1)];
u2to=[u2to;Infora2.MVopt(:,2)];
u3to=[u3to;Infora2.MVopt(:,3)];
u4to=[u4to;Infora2.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

C5=BottomSearch(C4,P);
X5=[C5(1) C5(2) -pi/2 0 0 0];
Infor5=nonlimpctr(X40',X5,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor5.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor5.Topt];
u1to=[u1to;Infor5.MVopt(:,1)];
u2to=[u2to;Infor5.MVopt(:,2)];
u3to=[u3to;Infor5.MVopt(:,3)];
u4to=[u4to;Infor5.MVopt(:,4)];
lst=[lst;getpathcells(C5,C4,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor5.Xopt(:,1),Infor5.Xopt(:,2),'ko','MarkerSize',10)

X50=[X5(1) X5(2) -pi 0 0 0];
Infora3=nonlimpctr(X5',X50,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora3.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora3.Topt];
u1to=[u1to;Infora3.MVopt(:,1)];
u2to=[u2to;Infora3.MVopt(:,2)];
u3to=[u3to;Infora3.MVopt(:,3)];
u4to=[u4to;Infora3.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infora3.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

C6=RightSearch(C5,P);
X6=[C6(1) C6(2) -pi 0 0 0];
Infor6=nonlimpctr(X50',X6,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor6.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor6.Topt];
u1to=[u1to;Infor6.MVopt(:,1)];
u2to=[u2to;Infor6.MVopt(:,2)];
u3to=[u3to;Infor6.MVopt(:,3)];
u4to=[u4to;Infor6.MVopt(:,4)];
lst=[lst;getpathcells(C6,C5,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor6.Xopt(:,1),Infor6.Xopt(:,2),'ko','MarkerSize',10)

C7=TopSearch(C6,P);
X7=[C7(1) C7(2) 0 0 0 0];
Infor7=nonlimpctr(X6',X7,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor7.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor7.Topt];
u1to=[u1to;Infor7.MVopt(:,1)];
u2to=[u2to;Infor7.MVopt(:,2)];
u3to=[u3to;Infor7.MVopt(:,3)];
u4to=[u4to;Infor7.MVopt(:,4)];
lst=[lst;getpathcells(C7,C6,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor7.Xopt(:,1),Infor7.Xopt(:,2),'ko','MarkerSize',10)

P=Environment(xig,yig,Ng,30+t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)

figure()
show(map)
hold on;
plot(Infor.Xopt(:,1),Infor.Xopt(:,2))
hold on;
plot(Infor2.Xopt(:,1),Infor2.Xopt(:,2))
hold on;
plot(Infor3.Xopt(:,1),Infor3.Xopt(:,2))
hold on;
plot(Infor4.Xopt(:,1),Infor4.Xopt(:,2))
hold on;
plot(Infor5.Xopt(:,1),Infor5.Xopt(:,2))
hold on;
plot(Infor6.Xopt(:,1),Infor6.Xopt(:,2))
hold on;
plot(Infor7.Xopt(:,1),Infor7.Xopt(:,2))
hold off;

figure();
plot(Ttot,u1to);

figure();
plot(Ttot,u2to);

figure();
plot(Ttot,u3to);

figure();
plot(Ttot,u4to);

function Cnex=TopSearch(Cpr,P)
Np=size(P,1);
it=Np-Cpr(2);
while it>1
    it=it-1;
    if P(it,Cpr(1))<0.1
        in=it;
        break
    end
    in=1;
end

if in>3
    Cnex=[Cpr(1) Np-in+1];
else
    Cnex=[Cpr(1) Np-in];
end
end

function Cnex=BottomSearch(Cpr,P)
Np=size(P,1);
it=Np-Cpr(2);
while it<Np
    it=it+1;
    if P(it,Cpr(1))<0.1
        in=it;
        break
    end
    in=Np-1;
end
if in<Np-3
    Cnex=[Cpr(1) Np-in+1];
else
    Cnex=[Cpr(1) Np-in];
end
end

function Cnex=RightSearch(Cpr,P)
Np=size(P,1);
jt=Cpr(1);
while jt<Np
    jt=jt+1;
    if P(Np-Cpr(2),jt)<0.1
        in=jt;
        break
    end
    in=Np-1;
end

Cnex=[in-1 Cpr(2)];

end

function Cnex=LeftSearch(Cpr,P)
Np=size(P,1);
jt=Cpr(1);
while jt>1
    jt=jt-1;
    if P(Np-Cpr(2),jt)<0.1
        in=jt;
        break
    end
    in=1;
end
if in==1
    Cnex=[in Cpr(2)];
else
    Cnex=[in-1 Cpr(2)];
end
end

function lst = getpathcells(Cstart,Cend,lst,Ng)

if Cstart(2)==Cend(2)
    if Cstart(1)<Cend(1)
        for k=Cstart(1):1:Cend(1)
            lst=[lst;[Ng-Cstart(2) k]];
        end
    else
        for k=Cend(1):1:Cstart(1)
            lst=[lst;[Ng-Cstart(2) k]];
        end
    end
elseif Cstart(1)==Cend(1)
    if Cstart(2)<Cend(2)
        for k=Cstart(2):1:Cend(2)
            lst=[lst;[Ng-k Cstart(1)]];
        end
    else
        for k=Cend(2):1:Cstart(2)
            lst=[lst;[Ng-k Cstart(1)]];
        end
    end
end
end
end

function Quadtwo(xig,yig,Ng,t0,lst)

P=Environment(xig,yig,Ng,t0,lst);
X0=[Ng;0;0;0;0;0];
map = occupancyMap(P);
figure()
show(map)

% [maxValue, linearIndex] = max(P(:));
% [i0,j0] = ind2sub(size(P), linearIndex);
C0=[xig yig];
C1t=BottomSearch(C0,P);
C1s=RightSearch(C1t,P);
C1=[C1s(1) C1s(2)];
X1=[C1(1) C1(2) 0 0 0 0];
Infor=nonlimpctr(X0,X1,20);
Ttot=Infor.Topt;
u1to=Infor.MVopt(:,1);
u2to=Infor.MVopt(:,2);
u3to=Infor.MVopt(:,3);
u4to=Infor.MVopt(:,4);
P=Environment(xig,yig,Ng,t0+Infor.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor.Xopt(:,1),Infor.Xopt(:,2),'ko','MarkerSize',10)


C2=LeftSearch(C1,P);
X2=[C2(1) C2(2) 0 0 0 0];
Infor2=nonlimpctr(X1',X2,15);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor2.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor2.Topt];
u1to=[u1to;Infor2.MVopt(:,1)];
u2to=[u2to;Infor2.MVopt(:,2)];
u3to=[u3to;Infor2.MVopt(:,3)];
u4to=[u4to;Infor2.MVopt(:,4)];

lst=[lst;getpathcells(C1,C2,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor2.Xopt(:,1),Infor2.Xopt(:,2),'ko','MarkerSize',10)

X20=[X2(1) X2(2) pi/2 0 0 0];
Infora=nonlimpctr(X2',X20,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora.Topt];
u1to=[u1to;Infora.MVopt(:,1)];
u2to=[u2to;Infora.MVopt(:,2)];
u3to=[u3to;Infora.MVopt(:,3)];
u4to=[u4to;Infora.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end),lst);

C3=TopSearch([X2(1)+1 X2(2)],P);
C3=[C2(1) C3(2)];
X3=[C3(1) C3(2) pi/2 0 0 0];
Infor3=nonlimpctr(X20',X3,15);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor3.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor3.Topt];
u1to=[u1to;Infor3.MVopt(:,1)];
u2to=[u2to;Infor3.MVopt(:,2)];
u3to=[u3to;Infor3.MVopt(:,3)];
u4to=[u4to;Infor3.MVopt(:,4)];
lst=[lst;getpathcells(C3,C2,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infor3.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor3.Xopt(:,1),Infor3.Xopt(:,2),'ko','MarkerSize',10)

X30=[X3(1) X3(2) pi 0 0 0];
Infora1=nonlimpctr(X3',X30,20);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora1.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora1.Topt];
u1to=[u1to;Infora1.MVopt(:,1)];
u2to=[u2to;Infora1.MVopt(:,2)];
u3to=[u3to;Infora1.MVopt(:,3)];
u4to=[u4to;Infora1.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end),lst);

C4=RightSearch(C3,P);
X4=[C4(1) C4(2) pi 0 0 0];
Infor4=nonlimpctr(X30',X4,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor4.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor4.Topt];
u1to=[u1to;Infor4.MVopt(:,1)];
u2to=[u2to;Infor4.MVopt(:,2)];
u3to=[u3to;Infor4.MVopt(:,3)];
u4to=[u4to;Infor4.MVopt(:,4)];
lst=[lst;getpathcells(C4,C3,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor4.Xopt(:,1),Infor4.Xopt(:,2),'ko','MarkerSize',10)

X40=[X4(1) X4(2) pi/2 0 0 0];
Infora2=nonlimpctr(X4',X40,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora2.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora2.Topt];
u1to=[u1to;Infora2.MVopt(:,1)];
u2to=[u2to;Infora2.MVopt(:,2)];
u3to=[u3to;Infora2.MVopt(:,3)];
u4to=[u4to;Infora2.MVopt(:,4)];

P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);
C5=BottomSearch(C4,P);
C5=[C4(1) C5(2)];
X5=[C5(1) C5(2) pi/2 0 0 0];
Infor5=nonlimpctr(X40',X5,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor5.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor5.Topt];
u1to=[u1to;Infor5.MVopt(:,1)];
u2to=[u2to;Infor5.MVopt(:,2)];
u3to=[u3to;Infor5.MVopt(:,3)];
u4to=[u4to;Infor5.MVopt(:,4)];
lst=[lst;getpathcells(C5,C4,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor5.Xopt(:,1),Infor5.Xopt(:,2),'ko','MarkerSize',10)

X50=[X5(1) X5(2) 0 0 0 0];
Infora3=nonlimpctr(X5',X50,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora3.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora3.Topt];
u1to=[u1to;Infora3.MVopt(:,1)];
u2to=[u2to;Infora3.MVopt(:,2)];
u3to=[u3to;Infora3.MVopt(:,3)];
u4to=[u4to;Infora3.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infora3.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

C6=LeftSearch(C5,P);
X6=[C6(1) C6(2) 0 0 0 0];
Infor6=nonlimpctr(X50',X6,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor6.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor6.Topt];
u1to=[u1to;Infor6.MVopt(:,1)];
u2to=[u2to;Infor6.MVopt(:,2)];
u3to=[u3to;Infor6.MVopt(:,3)];
u4to=[u4to;Infor6.MVopt(:,4)];
lst=[lst;getpathcells(C6,C5,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor6.Xopt(:,1),Infor6.Xopt(:,2),'ko','MarkerSize',10)

C7=TopSearch(C6,P);
X7=[C7(1) C7(2)+2 pi/2 0 0 0];
Infor7=nonlimpctr(X6',X7,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infor7.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infor7.Topt];
u1to=[u1to;Infor7.MVopt(:,1)];
u2to=[u2to;Infor7.MVopt(:,2)];
u3to=[u3to;Infor7.MVopt(:,3)];
u4to=[u4to;Infor7.MVopt(:,4)];
lst=[lst;getpathcells([X7(1) X7(2)],C6,lst,Ng)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor7.Xopt(:,1),Infor7.Xopt(:,2),'ko','MarkerSize',10)

P=Environment(xig,yig,Ng,30+t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)

figure()
show(map)
hold on;
plot(Infor.Xopt(:,1),Infor.Xopt(:,2))
hold on;
plot(Infor2.Xopt(:,1),Infor2.Xopt(:,2))
hold on;
plot(Infor3.Xopt(:,1),Infor3.Xopt(:,2))
hold on;
plot(Infor4.Xopt(:,1),Infor4.Xopt(:,2))
hold on;
plot(Infor5.Xopt(:,1),Infor5.Xopt(:,2))
hold on;
plot(Infor6.Xopt(:,1),Infor6.Xopt(:,2))
hold on;
plot(Infor7.Xopt(:,1),Infor7.Xopt(:,2))
hold off;

figure();
plot(Ttot,u1to);

figure();
plot(Ttot,u2to);

figure();
plot(Ttot,u3to);

figure();
plot(Ttot,u4to);


function Cnex=TopSearch(Cpr,P)
Np=size(P,1);
it=Np-Cpr(2);
while it>1
    it=it-1;
    if P(it,Cpr(1))<0.1
        in=it;
        break
    end
    in=1;
end

if in>3
    Cnex=[Cpr(1) Np-in+1];
else
    Cnex=[Cpr(1) Np-in];
end


end

function Cnex=BottomSearch(Cpr,P)
Np=size(P,1);
it=Np-Cpr(2);
while it<Np
    it=it+1;
    if P(it,Cpr(1))<0.1
        in=it;
        break
    end
    in=Np-1;
end
if in<Np-3
    Cnex=[Cpr(1)-1 Np-in+1];
else
    Cnex=[Cpr(1)-1 Np-in+1];
end
end

function Cnex=RightSearch(Cpr,P)
Np=size(P,1);
jt=Cpr(1);
while jt<Np
    jt=jt+1;
    if P(Np-Cpr(2),jt)<0.1
        in=jt;
        break
    end
    in=Np-1;
end

Cnex=[in-1 Cpr(2)];

end

function Cnex=LeftSearch(Cpr,P)
Np=size(P,1);
jt=Cpr(1);
while jt>1
    jt=jt-1;
    if P(Np-Cpr(2),jt)<0.1
        in=jt;
        break
    end
    in=1;
end
if in==1
    Cnex=[in Cpr(2)];
else
    Cnex=[in-1 Cpr(2)];
end
end

function lst = getpathcells(Cstart,Cend,lst,Ng)


if Cstart(2)==Cend(2)
    if Cstart(1)<Cend(1)
        for k=Cstart(1):1:Cend(1)
            lst=[lst;[Ng-Cstart(2) k]];
            
            
        end
    else
        for k=Cend(1):1:Cstart(1)
           
            lst=[lst;[Ng-Cstart(2) k]];

     
        end
    end
elseif Cstart(1)==Cend(1)
    if Cstart(2)<Cend(2)
        for k=Cstart(2):1:Cend(2)
            lst=[lst;[Ng-k Cstart(1)]];
           
        end
    else
        for k=Cend(2):1:Cstart(2)
            lst=[lst;[Ng-k Cstart(1)]];
            
        end
    end
end



end


end

function P = Environment(xig,yig,Ng,tsp,lst)

P=zeros(Ng);
P(Ng-yig,xig)=0.1;
t=1;
ig={[Ng-yig xig]};
sat={};

if size(lst,1)~=0
for p=1:size(lst,1)
    sat=[sat;[lst(p,1) lst(p,2)]];
    P(lst(p,1),lst(p,2))=-0.2;
end
end

while t<tsp
    
    if t<7
        for i=1:Ng
            for j=1:Ng
                for k=1:size(ig,1)
                    if isequal(ig{k},[i j]) && P(i,j)<0.7
                        P(i,j)=P(i,j)+0.05;
                        break
                    end
                end
                
            end
        end
    end
    if t<14
        for i=1:Ng
            for j=1:Ng
                if P(i,j)>0.7
                    sat=[sat;[i j]];
                end
                
            end
        end
    end
    
    if t>6
        for i=1:Ng
            for j=1:Ng
                for u=1:size(sat,1)
                    
                    if ~isequal(sat{u},[i j]) && P(i,j)>0.7
                        sat=[sat;[i j]];
                        break
                    end
                end
            end
        end
    end
    
    if t>6
        for i=1:Ng
            for j=1:Ng
                for n=1:size(ig,1)
                    if isequal(ig{n},[i j])
                        in=0;
                        for p=1:size(sat,1)
                            if isequal(sat{p},[i j])
                                in=1;
                                break
                            end
                            
                        end
                        if in==1 && P(i,j)>0.1
                            P(i,j)=P(i,j)-0.05;
                            break
                            
                        elseif in==0 && P(i,j)>0
                            
                            P(i,j)=P(i,j)+0.05;
                            break
                        end
                        
                        
                        
                    end
                end
            end
        end
    end
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[1 1])
            ind=1;
            break
        end
    end
    if ind==0 && (P(1,2)>=0.4|| P(2,1)>=0.4 || P(2,2)>=0.4)
        P(1,1)=0.1;
        ig=[ig;[1 1]];
        
    end
    
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[1 Ng])
            ind=1;
            break
        end
    end
    if ind==0 && (P(1,(Ng-1))>=0.4|| P(2,(Ng-1))>=0.4 || P(2,Ng)>=0.4)
        P(1,Ng)=0.1;
        ig=[ig;[1 Ng]];
        ind=1;
    end
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[Ng 1])
            ind=1;
            break
        end
    end
    if ind==0 && (P((Ng-1),1)>=0.4|| P((Ng-1),2)>=0.4 || P(Ng,2)>=0.4)
        P(Ng,1)=0.1;
        ig=[ig;[Ng 1]];
        ind=1;
    end
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[Ng Ng])
            ind=1;
            break
        end
    end
    if ind==0 && (P((Ng-1),(Ng-1))>=0.4|| P((Ng-1),Ng)>=0.4 || P(Ng,(Ng-1))>=0.4)
        
        P(Ng,Ng)=0.1;
        ig=[ig;[Ng Ng]];
        ind=1;
    end
    
    for i=2:Ng-1
        for j=2:Ng-1
            ind=0;
            for q=1:size(ig,1)
                if isequal(ig{q},[i j])
                    ind=1;
                    break
                end
            end
            if ind==0
                if (P((i-1),j-1)>=0.4 || P((i-1),j)>=0.4 ||P(i-1,j+1)>=0.4 || P(i,j-1)>=0.4 ||P(i,j+1)>=0.4 || P(i+1,j-1)>=0.4 || P((i+1),j)>=0.4 ||P((i+1),j+1)>=0.4)
                    P(i,j)=0.1;
                    ig=[ig;[i j]];
                    ind=1;
                    
                end
                
                
                
            end
        end
    end
    
    for i=2:Ng-1
        j=1;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P((i+1),1)>=0.4 ||P((i-1),1)>=0.4  || P((i-1),2)>=0.4 ||P(i,2)>=0.4 || P((i+1),2)>=0.4)
            P(i,1)=0.1;
            ig=[ig;[i 1]];
            ind=1;
        end
        
        j=Ng;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P(i-1,Ng)>=0.4 || P(i+1,Ng)>=0.4 || P(i-1,Ng-1)>=0.4 ||P(i,Ng-1)>=0.4 || P((i+1),Ng-1)>=0.4)
            P(i,Ng)=0.1;
            ig=[ig;[i Ng]];
            ind=1;
        end
    end
    
    for j=2:Ng-1
        i=1;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P(1,j-1)>=0.4 || P(1,j+1)>=0.4 || P(2,j-1)>=0.4 ||P(2,j)>=0.4 || P(2,j+1)>=0.4)
            
            P(1,j)=0.1;
            ig=[ig;[1 j]];
            ind=1;
        end
        
        i=Ng;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 && (P(Ng,j-1)>=0.4 || P(Ng,j+1)>=0.4 || P(Ng-1,j-1)>=0.4 ||P(Ng-1,j)>=0.4 || P(Ng-1,j+1)>=0.4)
            P(Ng,j)=0.1;
            ig=[ig;[Ng j]];
            ind=1;
        end
    end
    
    t=t+1;
end

for i=1:Ng
    for j=1:Ng
        if P(i,j)<0
            P(i,j)=0;
        end
    end
end

end

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




