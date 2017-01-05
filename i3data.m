%%% Vehicle data Car:

%kinda global
g = 9.81;
incl = 0.45; %.1736;
Vrange = 0:150; %%% In KM/H YOU STUPID FUCK! :D
a = 0; %start with this, update later

%kinda i3:
m = 1200;
Cd = 0.29;
A = 2.38;
CdA = Cd*A;
rhoAir = 1.2;
Cr = 0.006; % "sub 0.0065" maybe try with less to see.
% 175/60 R19 or 175/55 R20
tireD = 20*0.0245+2*0.175*0.55;%*2/3; % times 2/3 for effective real tire radius
tireR = tireD / 2;

motorToWheelRatio = 9.7;



for i=1:length(Vrange)
    Froll = Cr * m * g;
    Fdrag = Cd*A*rhoAir*(Vrange(i)/3.6)^2/2; % grab instantaneous V-thing.
    Fincl = m * g * incl;
    Fm = m * a;

    Ftrac(i) = Froll + Fdrag + Fincl + Fm; % maybe add Ffrict
end


%Torque speed curve for i3
load('i3speedTorque.mat');

i3wheelSpdTq(:,1) = i3speedTorque(:,1)*(2*pi/60)/9.7;
i3wheelSpdTq(:,2) = i3speedTorque(:,2)*9.7;

i3wheelSpdForce(:,1) = i3wheelSpdTq(:,1);
i3wheelSpdForce(:,2) = i3wheelSpdTq(:,2)/tireR;

i3machineSpeedPower = i3speedTorque(:,1)*(2*pi/60).*i3speedTorque(:,2);
i3wheelSpdkmh = i3wheelSpdForce(:,1) / (2*pi/60) * tireD * pi * 60 / 1000 ;


i3wheelSpdkmh10thsteps(:,1) = 0:0.1:150; %1 is speeds
i3wheelSpdkmh10thsteps(:,2) = interp1(i3wheelSpdkmh, i3wheelSpdForce(:,2),i3wheelSpdkmh10thsteps(:,1));
i3wheelLoadkmh10thsteps = interp1(Vrange,Ftrac,i3wheelSpdkmh10thsteps(:,1));

%%% Vehicle data Truck:

%fill this with fun info about a truck
mTruck = 60000; %or something like this?






%fix Vtruck
vTruck = 80;
% init vcar 80 km/h
iterStep = 1; %one is zero, just so you know. durrp
%vCar = zeros(1e5,1); %pre allocate, maybe dangerous?
%pCar = zeros(1e5,1); %pre allocate, maybe dangerous?
vCar(iterStep) = 80; % in km/h, remember to translate to m/s if you want to do anything useful
pCar(iterStep) = -(25+8); %25.25 meters + 2 car lengths

%f = ma shit
tStep = 0.0001;
%need to get car length + 2 car lengths in front (4 * 3m) for safe overtake
while pCar < 12;
    %for speed10=Vcar*10:1500 %maybe make this robust for different vmaxes
    iterStep;
    fDiff(iterStep) = i3wheelSpdkmh10thsteps(round(vCar(iterStep),1)*10,2) - (i3wheelLoadkmh10thsteps(round(vCar(iterStep),1)*10) + m*a); %; %% assume max 0.1 m/s^2, also HAX hAx
    fUsed(iterStep) = i3wheelSpdkmh10thsteps(round(vCar(iterStep),1)*10,2);
    fLoad(iterStep) = (i3wheelLoadkmh10thsteps(round(vCar(iterStep),1)*10) + m*a);
    a = fDiff(iterStep) / m;
    vCar(iterStep+1) = (vCar(iterStep)/3.6 + a*tStep)*3.6; %haxx haxx.
    vDiff = vCar(iterStep+1) - vTruck;
    pCar(iterStep+1) = pCar(iterStep) + vDiff/3.6*tStep; %because meters per second is important
    
    %go to next timestep
    iterStep=iterStep+1;
    %but update load in this step due to accel from previous no do this up
    %there
end

maLoad = movmean(fLoad,20);
maFdiff = movmean(fDiff,20);

%% plotting

figure

plot(i3wheelSpdkmh,i3wheelSpdForce(:,2),'DisplayName','Available')
hold on
plot(Vrange,Ftrac,'DisplayName','Steady state load')
% Create xlabel
xlabel({'Speed (km/h)'},'FontSize',11);
% Create ylabel
ylabel({'Wheel Force (N)'},'FontSize',11);

figure
plot(vCar(1:end-10),fUsed(1:end-9),'DisplayName','F_{used}')
hold on
plot(vCar(1:end-10),maLoad(1:end-9),'DisplayName','F_{load}')
plot(vCar(1:end-10),maFdiff(1:end-9),'DisplayName','F_{diff}')
% Create xlabel
xlabel({'Speed (km/h)'},'FontSize',11);
% Create ylabel
ylabel({'Wheel force (N)'},'FontSize',11);
legend1 = legend('show');
set(legend1,'Location','southwest');
% scatter(80,2319)
% scatter(80.25,5643)
% scatter(110,3814)
% scatter(110,2501)
% 
% figure
% plot(fLoad)
% hold on
% plot(fUsed)