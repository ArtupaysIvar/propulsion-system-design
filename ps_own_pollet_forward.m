clear; clc; close all;


MODE = 1; % hover

%% Param

% variables
D_data = [9, 10];
pitch_data = [45, 47];
m_bat = [0.55];
battery_voltage = 16.8;
battery_capacity_Ah = 6.5;

% Electrical parameters (assumed)
% usable_fraction = 0.80;
% eta_ESC_motor_assumed = 0.80;
usable_fraction = .8;
eta_ESC_motor_assumed = 1;
E_usable_Wh = battery_voltage*battery_capacity_Ah*usable_fraction;

% drone param
N_rotors = 4;
m = 1.2 + m_bat;
g = 9.81;
W = m*g;
TWR_required = 2;

D_m = D_data .* 0.0254;       % Diameter (m)
pitch_m = 4.7 .* 0.0254; % Pitch (m)

r_prime = 0.75;       % Representative blade radius
beta = pitch_m/D_m;
beta_prime = atan(pitch_m / (pi * D_m * r_prime)); % not in pollet
sigma_e = 0.0707; % not in pollet

T_hover_per_rotor = W/N_rotors;
T_design_per_rotor = TWR_required*W/N_rotors;

% Operating conditions
rho = 1.3;
V_inf = 10;       
RPM = 7000;   
n = RPM / 60; 
alpha = deg2rad(5); % Incidence angle (rad)

% Drag parameters
% Cd_frame = 1.0;           % assumed; replace with measured/CFD result
% Cl_frame = 1.0;           % assumed no airframe lift/downforce
% S_top = 0.15;             % m^2, ASSUMED projected area, replace
% S_front = 0.06;           % m^2, ASSUMED projected area, replace

% RUMUS J
if V_inf == 0 && MODE == 1
    CT0 = 0.02791 + 0.11867*beta ...
        + 0.27334*beta^2 - 0.28852*beta^3;
    CP0 = 0.01813 - 0.06218*beta ...
        + 0.35712*beta^2 - 0.23774*beta^3;

    rpm_hover = sqrt(T_hover_per_rotor/(CT0*rho*D_m^4))*60;
    n = rpm_hover / 60;
    
    Thrust = CT0*rho*n^2*D_m^4;    % N/rotor
    Pshaft = CP0*rho*n^3*D_m^5; % W/rotor

else
    J = V_inf / (n * D_m);
    J_axial = J * sin(alpha);

    CT0 = 0.02791 + 0.11867*beta ...
        + 0.27334*beta^2 - 0.28852*beta^3;
    CP0 = 0.01813 - 0.06218*beta ...
        + 0.35712*beta^2 - 0.23774*beta^3;

    CT_coeff = [0.02104, -0.23504, ...
        -0.06543 + 0.18677*beta^2, CT0];

    CP_coeff = [-0.1235, ...
        0.00343 + 0.07549*beta, CP0];
    
    % Find roots
    JT_roots = roots(CT_coeff);
    JP_roots = roots(CP_coeff);
    % Select smallest positive real roots
    JT_valid = JT_roots(abs(imag(JT_roots)) < 1e-8 & real(JT_roots) > 0);
    JP_valid = JP_roots(abs(imag(JP_roots)) < 1e-8 & real(JP_roots) > 0);
    J0T = min(real(JT_valid));
    J0P = min(real(JP_valid));

    CT_axial = 0.02791 - 0.06543*J_axial + 0.11867*beta ...
        + 0.27334*beta^2 - 0.28852*beta^3 ...
        + 0.02104*J_axial^3 - 0.23504*J_axial^2 + 0.18677*beta^2*J_axial;
    CP_axial = 0.01813 - 0.06218*beta + 0.00343*J_axial ...
        + 0.35712*beta^2 - 0.23774*beta^3 ...
        + 0.07549*beta*J_axial - 0.1235*J_axial^2;

    delta = (3/2) * cos(beta_prime) * ...
        (1 + (sigma_e / tan(beta_prime)) * ...
        (1 + sqrt(1 + 2*tan(beta_prime)/sigma_e)) * ...
        (1 - cos(alpha)));
  
    numerator = (J * cos(alpha) / (pi*r_prime))^2;
    eta_T = 1 + numerator * delta / ...
        (2 * (1 - J_axial / J0T));

    eta_P = 1 + numerator * delta / ...
        (2 * (1 - J_axial / J0P));

    CT_corrected = eta_T * CT_axial;
    CP_corrected = eta_P * CP_axial;

    Thrust = CT_corrected*rho*n^2*D_m^4;
    Pshaft = CP_corrected*rho*n^3*D_m^5;
end
disp(Thrust);
disp(Pshaft);

%% Hover calculations
% n_hover = rpm_hover/60;
% Pshaft_hover = CP0*rho*n_hover^3*D^5;

% Pelec_hover = Pshaft_hover/eta_ESC_motor_assumed;
Pelec_hover = Pshaft/eta_ESC_motor_assumed;
Ptotal_hover = N_rotors*Pelec_hover;
endurance_hover = E_usable_Wh/Ptotal_hover*60

%% additionals
% Q_static = Pshaft_static./(2*pi*n); % Nm/rotor
% Pelec_static = Pshaft_static/eta_ESC_motor_assumed;
% 
% I_static = Pelec_static/battery_voltage;
% 
% T_hover_per_rotor = W/N_rotors;
% T_design_per_rotor = TWR_required*W/N_rotors;
% 
% rpm_hover = sqrt(T_hover_per_rotor/(CT0*rho*D^4))*60;
% Pshaft_hover = interp1(rpm,Pshaft_static,rpm_hover,'linear',NaN);
% Pelec_hover = Pshaft_hover/eta_ESC_motor_assumed;
% Ptotal_hover = N_rotors*Pelec_hover+avionics_power;
% endurance_hover = E_usable_Wh/Ptotal_hover*60;
