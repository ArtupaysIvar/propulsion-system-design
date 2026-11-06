clear; clc; close all;
% variables
D_data = [9, 10];
pitch_data = [45, 47];
m_bat = [0.55, 0.6];
battery_voltage = 16.8;
battery_capacity_Ah = 6.5;

D_m = D_data .* 0.0254;  % Diameter (m)
pitch_m = pitch_data .* 0.0254; % Pitch (m)
beta = pitch_m/D_m;

% Electrical parameters (assumed)
usable_fraction = .8;
eta_ESC_motor_assumed = 1;
E_usable_Wh = battery_voltage*battery_capacity_Ah*usable_fraction;

% drone param
N_rotors = 4;
m = 1.2 + m_bat;
g = 9.81;
W = m*g;
TWR_required = 2;

T_hover_per_rotor = W/N_rotors;
T_design_per_rotor = TWR_required*W/N_rotors;
rho = 1.3;

functinprop_9045
CT0 = 0.02791 + 0.11867*beta ...
    + 0.27334*beta^2 - 0.28852*beta^3;
CP0 = 0.01813 - 0.06218*beta ...
    + 0.35712*beta^2 - 0.23774*beta^3;

rpm_hover = sqrt(T_hover_per_rotor/(CT0*rho*D_m^4))*60;
n = rpm_hover / 60;

Thrust = CT0*rho*n^2*D_m^4;    % N/rotor
Pshaft = CP0*rho*n^3*D_m^5; % W/rotor

disp(Thrust);
disp(Pshaft);

%% Hover calculations
% n_hover = rpm_hover/60;
% Pshaft_hover = CP0*rho*n_hover^3*D^5;

% Pelec_hover = Pshaft_hover/eta_ESC_motor_assumed;
Pelec_hover = Pshaft/eta_ESC_motor_assumed;
Ptotal_hover = N_rotors*Pelec_hover;
endurance_hover = E_usable_Wh/Ptotal_hover*60


% Drag parameters
% Cd_frame = 1.0;           % assumed; replace with measured/CFD result
% Cl_frame = 1.0;           % assumed no airframe lift/downforce
% S_top = 0.15;             % m^2, ASSUMED projected area, replace
% S_front = 0.06;           % m^2, ASSUMED projected area, replace
