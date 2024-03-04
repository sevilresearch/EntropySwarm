%%%%%%%%%%%%%%% Track Vehicle Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
b = 0.3; % HES 2019 0.5842 ; % [m] Effective Platform Width = Diagonal length
             % Actual Width = 0.3556 m
rNominal = 0.052959; % [m] Nominal Wheel Raduis

Vmax = 5; %HES 2019 0.43; %132/866.1417; % [m/s] Maximum speed of the vehicle

% wMax = Vmax/rNominal; % [rad/s] Maximum angular speed of wheels % NOT USED

rr = 1*rNominal; %% Effective vehicle right wheel 
rl = 1*rNominal; %% 1.06 %% Effective vehicle left wheel to represent inaccuracy in the vehicle

%%%%%%%%%%%%%%% Encoder Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
eTick =900; % 1040; % [ticks/m] number of ticks per 1 m of vehicle translation % from 22 [ticks/inch]


TauE = 0.5;
TauC = 0.6;   % Compass time Constant
%%
%%%%%%%%%%%%%%%%%% initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%
xIC = 150;
yIC = 0;
thetaIC = 90*(pi/180);

xIC2 = 25;
yIC2 = 0;
thetaIC2 = 90*(pi/180);

xIC3 = -100;
yIC3 = 0;
thetaIC3 = 90*(pi/180);
%%
%%%%%%%%%%%%%%%%%%% control parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
% KP1=20;     % 1 is velocity controller 3
% KP2=12;  %7 ->   % 2 is angle controller 6
% KI1=0.001;
% KI2=0.001; % 3 ->
% KD1=0;
% KD2=0; %0.5 -> 0.2

KP1=30;     % 1 is velocity controller
KP2=20;  %7 ->   % 2 is angle controller
KI1=10; %1;
KI2=0.001; %1; % 3 ->
KD1=0.1; %0.01;
KD2=0.1; %0.1; %0.5 ->

Tau1 = 0.5; %0.5;     %time constant of filter 1
Tau2 = 0.2; %0.2; %4;       %time constant of filter 2


KP1=40;     % 1 is velocity controller
KP2=7;  %7 ->   % 2 is angle controller
KI1=10;
KI2=1; % 3 ->
KD1=3;
KD2=0.5; %0.5 ->
Tau1 = 0.1;     %time constant of filter 1
Tau2 = 1;       %time constant of filter 2



%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
rp1 =  0.3; % 1 -> [m] proximity circle to start slowing down or VT
rp2 =  0.7;  % [m] radius of wayPoint proximity circle to switch to the next wayPoint
Vcom = Vmax; % [m/s] used when constant speed is commanded

%%%%%%%%%%%%%%%%%%%%%%% SAMPLE TIME %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters for sample time

Tcompass=0.4;   % sample time for digital compass

Tguidance=0.1; % HAKKI 1.6;    % Sample time of guidance module

Tsample = 1/10;   %sampling rate -> 0.5 % update rate of the model

TsampleEncoder = 1/100; % 0.1 [s] Encoder sample time

Tmodel = Tsample;  % Update rate of the whole model

Tlrf=1/10; % sample time of the laser range finder 

Tgps= Tcompass;   % Sample time of the GPS receiver

Tworkspace=1/5; % Sample time for to workspace blocks

%%%%%%%%%%%%%%%%%%%%%%% WAYPOINTS AND OBSTACLES %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%  Defining an Obstacle
obs_radius_array=[0.1;0.1;0.1;0.1;0.1]; %[0.1;0.1]
obs_pos_array=[4.2 2;5 6;7.1 5;3.5 5;2.5 3]; %[4.2 2;3.8 3;5 6;7.1 5;3.5 5;2.5 3;3.5 1.5]; %[8 7]; %[7 8]; %[2 2]; %[7 8]; %[2.14 0.295;1.81 2.48];
obsTimeInterval_array = [0 50000;0 50000;0 50000;0 50000;0 50000]; %[0 50000;0 50000]% obstacle will be visible from time 5 sec to 50 sec


%%%  Defining waypoints
x_waypoint=[0 4 6 7 3 3 4]; %[5 8]; %[2 4 2 0 2]; %[5]; %[2 4 2 0 2]; %[2 4 3 2 1 0 0]; %[2 4 2 0 2]; %[0 2 4 2];
y_waypoint=[500 4 7 3 7 2 1]; %[5 10]; %[2 8 10 5 2]; %[5]; %[2 8 10 5 2]; %[2 8 10 10 8 6 0]; %[2 8 10 5 2]; %[5 10 8 2];

a=x_waypoint(1)+10;
b=y_waypoint(1);

a2=x_waypoint(1);
b2=y_waypoint(1);

a3=x_waypoint(1)-10;
b3=y_waypoint(1);

target_proximity=0.7;

%eval('look_up_table_create')
%eval('load lookup_table_data');