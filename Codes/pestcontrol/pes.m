clc;clear;close all;

% % % prompt = "Enter the Coordinates of ignition point as a 1x2 matrix with []-";
% % C = input(prompt);
% %
% % prompt3 = "Enter the Grid size of the environment";
% % Ng = input(prompt3);
% %
% % fprintf('x(1)-x inertial coordinate of COM \nx(2)-y inertial coordinate of COM \nx(3)-theta-thrust direction \nx(4)-vx-velocity of x \nx(5)-vy-velocity of y \nx(6)-omega-angular velocity of theta \n')
% % X0 = input('Enter the Initial position and Orientation of the UAV ');

C=[26 21];
Ng=40;
lst=[];
xig=C(1);
yig=C(2);
t0=15;

P=Pestcontrol(xig,yig,Ng,t0,lst);
X0=[Ng;0;0;0;0;0];
map = occupancyMap(P);
figure()
show(map)

C0=[xig yig];
C1t=BottomSearch(C0,P);
C1=LeftSearch(C1t,P);
C1=[C1(1) C1(2)];
X1=[C1(1) C1(2) -pi 0 0 0];
Infor=nonlimpc(X0,X1,20);
Xplo=Infor.Xopt(:,1);
Yplo=Infor.Xopt(:,2);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor.Xopt(:,1),Infor.Xopt(:,2),'ko','MarkerSize',10)

C2=RightSearch(C1,P);
X2=[C2(1) C2(2) -pi 0 0 0];
Infor2=nonlimpc(X1',X2,20);
Xplo=[Xplo;Infor2.Xopt(:,1)];
Yplo=[Yplo;Infor2.Xopt(:,2)];
lst=getpathcells(C2,C1,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor2.Xopt(:,1),Infor2.Xopt(:,2),'ko','MarkerSize',10)

X20=[X2(1) X2(2) pi/2 0 0 0];
Infora=nonlimpc(X2',X20,5);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end),lst);
C3=TopSearch(C2,P);

X3=[C3(1) C3(2) pi/2 0 0 0];
Infor3=nonlimpc(X20',X3,20);
Xplo=[Xplo;Infor3.Xopt(:,1)];
Yplo=[Yplo;Infor3.Xopt(:,2)];
lst=getpathcells(C3,C2,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infor3.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor3.Xopt(:,1),Infor3.Xopt(:,2),'ko','MarkerSize',10)

X30=[X3(1) X3(2) 0 0 0 0];
Infora1=nonlimpc(X3',X30,10);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end),lst);
C4=LeftSearch(C3,P);

X4=[C4(1) C4(2) 0 0 0 0];
Infor4=nonlimpc(X30',X4,10);
Xplo=[Xplo;Infor4.Xopt(:,1)];
Yplo=[Yplo;Infor4.Xopt(:,2)];
lst=getpathcells(C4,C3,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor4.Xopt(:,1),Infor4.Xopt(:,2),'ko','MarkerSize',10)

X40=[X4(1) X4(2) -pi/2 0 0 0];
Infora2=nonlimpc(X4',X40,5);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);
C5=BottomSearch(C4,P);
X5=[C5(1) C5(2) -pi/2 0 0 0];
Infor5=nonlimpc(X40',X5,10);
Xplo=[Xplo;Infor5.Xopt(:,1)];
Yplo=[Yplo;Infor5.Xopt(:,2)];
lst=getpathcells(C5,C4,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor5.Xopt(:,1),Infor5.Xopt(:,2),'ko','MarkerSize',10)

X50=[X5(1) X5(2) -pi 0 0 0];
Infora3=nonlimpc(X5',X50,5);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infora2.Topt(end)+Infora3.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end),lst);

C6=RightSearch(C5,P);
X6=[C6(1) C6(2) -pi 0 0 0];
Infor6=nonlimpc(X50',X6,10);
Xplo=[Xplo;Infor6.Xopt(:,1)];
Yplo=[Yplo;Infor6.Xopt(:,2)];
lst=getpathcells(C6,C5,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor6.Xopt(:,1),Infor6.Xopt(:,2),'ko','MarkerSize',10)

X7=[X6(1) X6(2)+1 pi 0 0 0];
Infor7=nonlimpc(X6',X7,5);
Xplo=[Xplo;Infor7.Xopt(:,1)];
Yplo=[Yplo;Infor7.Xopt(:,2)];
C7=[X7(1) X7(2)];
lst=getpathcells(C7,C6,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end),lst);

map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor7.Xopt(:,1),Infor7.Xopt(:,2),'ko','MarkerSize',10)

C8=LeftSearch(C7,P);

X8=[C8(1) C8(2) pi 0 0 0];
Infor8=nonlimpc(X7',X8,10);
Xplo=[Xplo;Infor8.Xopt(:,1)];
Yplo=[Yplo;Infor8.Xopt(:,2)];
lst=getpathcells(C8,C7,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor8.Xopt(:,1),Infor8.Xopt(:,2),'ko','MarkerSize',10)

X9=[X8(1) X8(2)+1 0 0 0 0];
C9=[X9(1) X9(2)];
Infor9=nonlimpc(X8',X9,5);
Xplo=[Xplo;Infor9.Xopt(:,1)];
Yplo=[Yplo;Infor9.Xopt(:,2)];
lst=getpathcells(C9,C8,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor9.Xopt(:,1),Infor9.Xopt(:,2),'ko','MarkerSize',10)

C10=RightSearch(C9,P);
C10=[C10(1)-2 C10(2)];
X10=[C10(1) C10(2) 0 0 0 0];
Infor10=nonlimpc(X9',X10,15);
Xplo=[Xplo;Infor10.Xopt(:,1)];
Yplo=[Yplo;Infor10.Xopt(:,2)];
lst=getpathcells(C10,C9,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor10.Xopt(:,1),Infor10.Xopt(:,2),'ko','MarkerSize',10)

X11=[X10(1) X10(2)+1 pi 0 0 0];
Infor11=nonlimpc(X10',X11,5);
Xplo=[Xplo;Infor11.Xopt(:,1)];
Yplo=[Yplo;Infor11.Xopt(:,2)];
C11=[X11(1) X11(2)];
lst=getpathcells(C11,C10,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor11.Xopt(:,1),Infor11.Xopt(:,2),'ko','MarkerSize',10)

C12=LeftSearch(C11,P);
X12=[C12(1) C12(2) pi 0 0 0];
Infor12=nonlimpc(X11',X12,10);
Xplo=[Xplo;Infor12.Xopt(:,1)];
Yplo=[Yplo;Infor12.Xopt(:,2)];
lst=getpathcells(C12,C11,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor12.Xopt(:,1),Infor12.Xopt(:,2),'ko','MarkerSize',10)

X13=[X12(1) X12(2)+1 0 0 0 0];
C13=[X13(1) X13(2)];
Infor13=nonlimpc(X12',X13,5);
Xplo=[Xplo;Infor13.Xopt(:,1)];
Yplo=[Yplo;Infor13.Xopt(:,2)];
lst=getpathcells(C13,C12,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor13.Xopt(:,1),Infor13.Xopt(:,2),'ko','MarkerSize',10)

C14=RightSearch(C13,P);
C14=[C14(1) C14(2)];
X14=[C14(1) C14(2) 0 0 0 0];
Infor14=nonlimpc(X13',X14,15);
Xplo=[Xplo;Infor14.Xopt(:,1)];
Yplo=[Yplo;Infor14.Xopt(:,2)];
lst=getpathcells(C14,C13,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor14.Xopt(:,1),Infor14.Xopt(:,2),'ko','MarkerSize',10)

dist=abs(C14(1)-C13(1));
for i=1:4
C15=[C14(1) C14(2)+1];
X15=[C15(1) C15(2) -pi 0 0 0];
Infor15=nonlimpc(X14',X15,3);
Xplo=[Xplo;Infor15.Xopt(:,1)];
Yplo=[Yplo;Infor15.Xopt(:,2)];
lst=getpathcells(C15,C14,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor15.Xopt(:,1),Infor15.Xopt(:,2),'ko','MarkerSize',10)

C16=[C15(1)-dist C15(2)];
X16=[C16(1) C16(2) -pi 0 0 0];
Infor16=nonlimpc(X15',X16,10);
Xplo=[Xplo;Infor16.Xopt(:,1)];
Yplo=[Yplo;Infor16.Xopt(:,2)];
lst=getpathcells(C16,C15,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor16.Xopt(:,1),Infor16.Xopt(:,2),'ko','MarkerSize',10)

C17=[C16(1) C16(2)+1];
X17=[C17(1) C17(2) 0 0 0 0];
Infor17=nonlimpc(X16',X17,3);
Xplo=[Xplo;Infor17.Xopt(:,1)];
Yplo=[Yplo;Infor17.Xopt(:,2)];
lst=getpathcells(C17,C16,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor17.Xopt(:,1),Infor17.Xopt(:,2),'ko','MarkerSize',10)

C18=[C17(1)+dist C17(2)];
X18=[C18(1) C18(2) 0 0 0 0];
Infor18=nonlimpc(X17',X18,10);
Xplo=[Xplo;Infor18.Xopt(:,1)];
Yplo=[Yplo;Infor18.Xopt(:,2)];
lst=getpathcells(C18,C17,lst,Ng);
P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end)+Infor18.Topt(end),lst);
map = occupancyMap(P);
figure()
show(map)
hold on;
plot(Infor18.Xopt(:,1),Infor18.Xopt(:,2),'ko','MarkerSize',10)

C14(1)=C18(1);
C14(2)=C18(2);
X14=[C14(1) C14(2) 0 0 0 0];
end
% C19=TopSearch(C18,P);
% C19=[C19(1) C19(2)-2];
% X19=[C19(1) C19(2) 0 0 0 0];
% Infor19=nonlimpc(X18',X19,15);
% lst=getpathcells(C19,C18,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end)+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end)+Infor18.Topt(end)+Infor19.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor19.Xopt(:,1),Infor19.Xopt(:,2),'ko','MarkerSize',10)


% tlo=t0+Infor.Topt(end)+Infor2.Topt(end)+Infora.Topt(end)+Infora1.Topt(end)+Infor3.Topt(end)+Infor4.Topt(end)+Infor5.Topt(end)+Infora3.Topt(end)+Infor6.Topt(end)+Infor7.Topt(end)+Infor8.Topt(end)+Infor9.Topt(end)+Infor10.Topt(end);
% d=C10(1)-C9(1);
% 
% h=15;
% for t=1:6
%    
% C11=[C10(1) C10(2)+d-2];
% X11=[C11(1) C11(2) pi 0 0 0];
% Infor11=nonlimpc(X10',X11,h);
% lst=getpathcells(C11,C10,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end),lst);
% map = occupancyMap(P);
% figure() 
% show(map)
% hold on;
% plot(Infor11.Xopt(:,1),Infor11.Xopt(:,2),'ko','MarkerSize',10)
%  
% C12=[C11(1)-d C11(2)];
% X12=[C12(1) C12(2) pi 0 0 0];
% Infor12=nonlimpc(X11',X12,h);
% lst=getpathcells(C12,C11,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor12.Xopt(:,1),Infor12.Xopt(:,2),'ko','MarkerSize',10)
% 
% C13=[C12(1) C12(2)-d+1];
% X13=[C13(1) C13(2) 0 0 0 0];
% Infor13=nonlimpc(X12',X13,h);
% lst=getpathcells(C13,C12,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor13.Xopt(:,1),Infor13.Xopt(:,2),'ko','MarkerSize',10)
% 
% C14=[C13(1)+d C13(2)];
% X14=[C14(1) C14(2) 0 0 0 0];
% Infor14=nonlimpc(X13',X14,h);
% lst=getpathcells(C14,C13,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor14.Xopt(:,1),Infor14.Xopt(:,2),'ko','MarkerSize',10)
% 
% d=d-1;
% C10=C14;
% h=h-1;
% end

% C15=[C14(1) C14(2)+d-2];
% X15=[C15(1) C15(2) pi 0 0 0];
% Infor15=nonlimpc(X14',X15,5);
% lst=getpathcells(C15,C14,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end),lst);
% map = occupancyMap(P);
% figure() 
% show(map)
% hold on;
% plot(Infor15.Xopt(:,1),Infor15.Xopt(:,2),'ko','MarkerSize',10)
%  
% C16=[C15(1)-d-1 C15(2)];
% X16=[C16(1) C16(2) pi 0 0 0];
% Infor16=nonlimpc(X15',X16,9);
% lst=getpathcells(C16,C15,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor16.Xopt(:,1),Infor16.Xopt(:,2),'ko','MarkerSize',10)
% 
% C17=[C16(1) C16(2)-2];
% X17=[C17(1) C17(2) 0 0 0 0];
% Infor17=nonlimpc(X16',X17,3);
% lst=getpathcells(C17,C16,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor17.Xopt(:,1),Infor17.Xopt(:,2),'ko','MarkerSize',10)
% 
% C18=[C17(1)+d C17(2)];
% X18=[C18(1) C18(2) 0 0 0 0];
% Infor18=nonlimpc(X17',X18,9);
% lst=getpathcells(C18,C17,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end)+Infor18.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor18.Xopt(:,1),Infor18.Xopt(:,2),'ko','MarkerSize',10)
% 
% C19=[C18(1) C18(2)+1];
% X19=[C19(1) C19(2) 0 0 0 0];
% Infor19=nonlimpc(X18',X19,3);
% lst=getpathcells(C19,C18,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end)+Infor18.Topt(end)+Infor19.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor19.Xopt(:,1),Infor19.Xopt(:,2),'ko','MarkerSize',10)
% 
% C20=[C19(1)-d C19(2)];
% X20=[C20(1) C20(2) 0 0 0 0];
% Infor20=nonlimpc(X19',X20,8);
% lst=getpathcells(C20,C19,lst,Ng);
% P=Pestcontrol(xig,yig,Ng,tlo+Infor11.Topt(end)+Infor12.Topt(end)+Infor13.Topt(end)+Infor14.Topt(end)+Infor15.Topt(end)+Infor16.Topt(end)+Infor17.Topt(end)+Infor18.Topt(end)+Infor19.Topt(end)+Infor20.Topt(end),lst);
% map = occupancyMap(P);
% figure()
% show(map)
% hold on;
% plot(Infor20.Xopt(:,1),Infor20.Xopt(:,2),'ko','MarkerSize',10)

figure()
plot(Xplo,Yplo)

figure()
plot(Xplo,Yplo,'LineWidth',2)


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

if Cpr==Cnex
    Cnex=BottomSearch([Cpr(1)+1 Cpr(2)],P);
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

Cnex=[in Cpr(2)];

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
