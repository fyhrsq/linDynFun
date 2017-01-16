%%%%% A FS car tries to accelerate and a driver is heavier than the other
%%%%% so.... here is some seach or something.
%%% Vehicle data Car:

%kinda global
g = 9.81;
incl = 0;
Vrange = 0:0.01:150; %%% In KM/H YOU STUPID FUCK! :D
a = 0; %start with this, update later

for mdiff = 1:5:200
m = 600+mdiff;
Cd = 0.29;
A = 2.38;
CdA = Cd*A;
rhoAir = 1.2;
Cr = 0.006; % "sub 0.0065" maybe try with less to see.
tireD = 18*0.0245; %kinda hoosier R25 10" ?
tireR = tireD / 2;

motorToWheelRatio = 9.7; %gear ratio between motor and driveshaft
%%% add real gears later

%%% Road load calcs
for i=1:length(Vrange)
%     Froll = Cr * m * g;
%     Fdrag = Cd*A*rhoAir*(Vrange(i)/3.6)^2/2; % grab instantaneous V-thing.
%     Fincl = m * g * incl;
%     %Fm = m * a; %yeah this... yeah... dealt with in the loop
    Ftrac(i) = 0; %Froll + Fdrag + Fincl; %add friction limit
end


%Fastest Accel FScar with 25 kW
omegaMachine = Vrange / 3.6 * tireD * pi * motorToWheelRatio;

power = 25e3; %25 kw

tq = power ./ omegaMachine; %machine torque

%roadload is Ftrac
tqRoadForce = tq / (tireR * motorToWheelRatio / 3.6);

% init vcar 80 km/h
iterStep = 1; %one is zero, just so you know. durrp
%vCar = zeros(1e5,1); %pre allocate, maybe dangerous?
%pCar = zeros(1e5,1); %pre allocate, maybe dangerous?
vCar(iterStep,mdiff) = 0; % in km/h, remember to translate to m/s if you want to do anything useful
pCar(iterStep,mdiff) = 0; %25.25 meters + 2 car lengths

%f = ma shit
tStep = 0.0001;
%need to get car length + 2 car lengths in front (4 * 3m) for safe overtake
while pCar(:,mdiff) < 75;
    %for speed10=Vcar*10:1500 %maybe make this robust for different vmaxes
    iterStep;
    %%% semihax to avoid unpleasant things
    if vCar(iterStep) < 1;
        vLookup = 2;
    else
        vLookup = round(vCar(iterStep,mdiff),1)*10;
    end
    loadDiff = tqRoadForce(vLookup); %ignore atm - Ftrac(vLookup);
    fDiff(iterStep,mdiff) = loadDiff; 
    fUsed(iterStep,mdiff) = tqRoadForce(vLookup);
    fLoad(iterStep,mdiff) = Ftrac(vLookup) + m*a; %here the ma can be, because plots.
    a = fDiff(iterStep,mdiff) / m;
    vCar(iterStep+1,mdiff) = (vCar(iterStep,mdiff)/3.6 + a*tStep)*3.6; %really should be using m/s
    vDiff = vCar(iterStep+1,mdiff); % - vTruck; %i guess keep if you wanna race against something
    pCar(iterStep+1,mdiff) = pCar(iterStep,mdiff) + vCar(iterStep,mdiff)/3.6*tStep; %because meters per second is important
    
    %go to next timestep
    iterStep=iterStep+1;
    %but update load in this step due to accel from previous no do this up
    %there
end
%% add the coast down
a=0;
fUsed(iterStep,mdiff) = Ftrac(vLookup);
fLoad(iterStep,mdiff) = Ftrac(vLookup) + m*a;

end


for j = 1:length(vCar(1,:))
    loopVar = 3;
    while vCar(loopVar,j) > 1
        loopVar = loopVar +1;
    end
    finalTstep(j) = loopVar;
end
        


%% plotting

% figure
% 
% plot(Vrange,tqRoadForce,'DisplayName','Available')
% hold on
% plot(Vrange,Ftrac,'DisplayName','Steady state load')
% % Create xlabel
% xlabel({'Speed (km/h)'},'FontSize',11);
% % Create ylabel
% ylabel({'Wheel Force (N)'},'FontSize',11);
% plot(vCar(1:end),fUsed(1:end),'DisplayName','F_{used}')
% plot(vCar(1:end),fLoad(1:end),'DisplayName','F_{load}')

% figure
% plot(vCar(1:end),fUsed(1:end),'DisplayName','F_{used}')
% hold on
% plot(vCar(1:end),fLoad(1:end),'DisplayName','F_{load}')
% plot(vCar(1:end),fDiff(1:end),'DisplayName','F_{diff}')
% % Create xlabel
% xlabel({'Speed (km/h)'},'FontSize',11);
% % Create ylabel
% ylabel({'Wheel force (N)'},'FontSize',11);
% legend1 = legend('show');
% set(legend1,'Location','southwest');
% scatter(80,2319)
% scatter(80.25,5643)
% scatter(110,3814)
% scatter(110,2501)
% 
% figure
% plot(fLoad)
% hold on
% plot(fUsed)