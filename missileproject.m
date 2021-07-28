% SAM MISSILE TARGET TRAJECTORY PROBLEM
% ADE FSIM & AI INTERNSHIP 
% VAISHNAV.R 1NT18AE056 , PRAJWAL RODRIGUES 1NT18AE032 , RITHIK MARC 1NT18AE042
% SUBMITTED TO MR.SURAJ CHAURASIYA

 

%INITIAL CONDITIONS
Rt10 = 20000;       %ft
Rt20 = 20000;       %ft
Vt0 = 2900;         %ft/sec
Beta0 = 0;           %deg
Rm10 = 0;           %ft
Rm20 = 0;           %ft     
Vm10 = 0;           %ft/sec in vector 2 direction
Vm20 = 3900;        %ft/sec in vector 2 direction
nc = 0;

%TIME CONDITIONS
gaptime = 0.08;     %Time Between Which We Have No Information On The Target 
endtime = 13.5;     %sec
time = [0:gaptime:endtime];

%EXECUTION LINE FOR SOLVING THE ORDINARY DIFFERENTIAL EQUATION (ODE23)
%MATRIX INPUTS ARE TIME (0 TO RUNTIME) AND INITIAL CONDITIONS

[t,x] = ode23(@PropNav,time,[Rt10;Rt20;Vt0;Beta0;Rm10;Rm20;Vm10;Vm20;nc]); 

%EXTRACTING LINE FROM ODE23 EXECUTION
Rt1 = x(:,1);
Rt2 = x(:,2);
Rm1 = x(:,5);
Rm2 = x(:,6);
Vm1 = x(:,7);
Vm2 = x(:,8);
nc2 = x(:,9);

%CALCULATE THE MISS DISTANCE
Rtm1 = Rt1-Rm1;
Rtm2 = Rt2-Rm2;
Rtmsq = Rtm1.^2 + Rtm2.^2;
Rtm = sqrt(Rtmsq);
miss_distance = min(Rtm);



% PLOTTING THE DISTANCE BETWEEN MISSILE AND AIRCRAFT
% FIGURE(1)
subplot(2,2,1)
plot(t,Rtm,'g')
xlabel('Time (sec)')                % X Coordinate Label
ylabel('Separation (feet)')         % Y Coordinate Label
title('MISSILE TO TARGET DISTANCE')

% PLOTTING THE GODS-EYE TRAJECTORIES OF MISSILE AND AIRCRAFT
% FIGURE(2)
subplot(2,2,2)
plot(Rm1,Rm2,'r',Rt1,Rt2,'g')
xlabel('DownRange (feet)')                % X Coordinate Label
ylabel('CrossRange (feet)')         % Y Coordinate Label
title('GODS-EYE TRAJECTORY')
text(1000,5000,'Missile')
text(16000,19000,'Target')


fprintf('\n THE MISS DISTANCE FOR GAIN = 2.2 , TERMINAL GAIN 4 IS %G FEET.',miss_distance)


