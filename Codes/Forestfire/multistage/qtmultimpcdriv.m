clc;clear;close all;

C=[23 21];
Ng=40;
lst=[];
xig=C(1);
yig=C(2);
t0=30;

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
Infor=nonlimpcmultitr(X0,X1,20);
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
Infor2=nonlimpcmultitr(X1',X2,15);
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
Infora=nonlimpcmultitr(X2',X20,10);
s=size(Ttot,1);
Ttot=[Ttot;Ttot(end)*ones(size(Infora.Topt,1),1)];
Ttot=Ttot+[zeros(s,1);Infora.Topt];
u1to=[u1to;Infora.MVopt(:,1)];
u2to=[u2to;Infora.MVopt(:,2)];
u3to=[u3to;Infora.MVopt(:,3)];
u4to=[u4to;Infora.MVopt(:,4)];
P=Environment(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end),lst);

C3=TopSearch(X2,P);
% C3=TopSearch([X2(1)+1 X2(2)],P);
C3=[C2(1) C3(2)];
X3=[C3(1) C3(2) pi/2 0 0 0];
Infor3=nonlimpcmultitr(X20',X3,15);
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
Infora1=nonlimpcmultitr(X3',X30,20);
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
Infor4=nonlimpcmultitr(X30',X4,10);
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
Infora2=nonlimpcmultitr(X4',X40,10);
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
Infor5=nonlimpcmultitr(X40',X5,15);
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
Infora3=nonlimpcmultitr(X5',X50,10);
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
Infor6=nonlimpcmultitr(X50',X6,15);
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
X7=[C7(1) C7(2) pi/2 0 0 0];
Infor7=nonlimpcmultitr(X6',X7,15);
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
    Cnex=[in Cpr(2)];
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


