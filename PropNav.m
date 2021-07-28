
function xdot = PropNav(t,x)


% INPUT INITIAL STATE CONDITIONS
Rt1 = x(1);
Rt2 = x(2);
Vt = x(3);
Beta = x(4);
Rm1 = x(5);
Rm2 = x(6);
Vm1 = x(7);
Vm2 = x(8);
nc = x(9);
xdot = zeros(size(x));

% GAIN INPUTS TO THE MODEL 
Npr = 2.2;                       %PROPORTIONAL NAVIGATION CONSTANT  
TermNpr = 4;                   %TERMINAL NAVIGATION CONSTANT
TermRange = 4000;              %TERMINAL RANGE
maxg = 10;                     % MAX G-FORCE MISSILE HARDWARE CAN SUSTAIN

% FOUR MANEUVER INPUTS TO THE MODEL 
% NO MANEUVER
 nt=0; 

% SIDE TO SIDE MANEUVER 5G
% nt=32*5*sin(T);

% LATE 5G TURNOUT
%if t<5
    %nt=0;
%else 
    %nt=32*5;
%end

% LATE 3G TURN- IN 
%if t<0
%    nt=0; 
%else
%    nt=-32*3;
%end



%STATE EQN CALCULATONS
Rt1dot = -Vt*cos(Beta);
Rt2dot = Vt*sin(Beta);
Vtdot = 0;
Betadot = nt/Vt;
Rm1dot = Vm1;
Rm2dot = Vm2;

%CALCULATION OF LAMBDA
Rtm1 = Rt1-Rm1;
Rtm2 = Rt2-Rm2;
lambda = atan2(Rtm2,Rtm1);

%CALCULATION OF  nc
Vt1 = -Vt*cos(Beta);
Vt2 = Vt*sin(Beta);
Vtm1 = Vt1-Vm1; 
Vtm2 = Vt2- Vm2;
RtmSq =Rtm1^2 + Rtm2^2;
Rtm = sqrt(RtmSq);

if Rtm <= -3 %LAMDA LOGIC FOR TERMINAL RANGE ZERO HANDLING
    lambdadot = 0;
    Vc = 0;
    
else
    lambdadot = (Rtm1*Vtm2-Rtm2*Vtm1)/RtmSq;
    Vc= -(Rtm1*Vtm1 + Rtm2*Vtm2)/Rtm;
end    

% ENTER TERMINAL PHASE (HIGHER GAIN) IF WITHIN TERMINAL RANGE
if Rtm < TermRange
    Npr = TermNpr;                  %REPLACING THE MIDCOURSE GAIN WITH TERMINAL GAIN
end
nc = Npr*Vc*lambdadot;

if abs(nc) >32*maxg
    nc = 32*maxg*sign(nc);
end

Vm1dot = -nc*sin(lambda);
Vm2dot = nc*cos(lambda);

xdot= [Rt1dot; Rt2dot; Vtdot; Betadot; Rm1dot; Rm2dot; Vm1dot; Vm2dot; nc;];
