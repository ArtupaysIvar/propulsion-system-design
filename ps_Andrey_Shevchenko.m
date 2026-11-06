%% Multirotor Propulsion System Sizing & Verification Script
% Based on the 2026 International Russian Smart Industry Conference paper formulas

close all; clear; clc;

%% 1. Input Specifications & Constants
W = 15.0;            % [N] Total aircraft weight 
TWR = 3.0;           % Target Thrust-to-Weight Ratio (e.g., 3 for aerial photography)
N_rotors = 4;        % Number of rotors (e.g., 4 for quadcopter)
rho = 1.225;         % [kg/m^3] Air density at sea level

% Propeller Aerodynamic Assumptions
Ct = 0.11;           % Dimensionless thrust coefficient (typical value: 0.05 - 0.15)
n_rpm = 6500;        % [RPM] Approximate operational propeller speed in hover
M_prop = 0.25;       % [N*m] Aerodynamic torque load on the propeller

% Motor & ESC Specifications
Kv_rpm_v = 920;      % [RPM/V] Motor velocity constant 
I_max = 25;          % [A] Maximum continuous motor current
I_mot_cont = 25;     % [A] Maximum continuous current of the electric motor

%% 2. Propeller Selection Calculations (Section II-A)

% Eq (1): Total aircraft thrust required based on target TWR
T_AC = TWR * W; 

% Eq (2): Thrust per propeller (using N_rotors instead of the hardcoded median 5 for accuracy)
T_prop = T_AC / N_rotors; 

% Convert RPM to revs per second for aerodynamic equations
n_rps = n_rpm / 60; 

% Eq (3): Required propeller diameter (in meters)
D_prop = ( T_AC / (Ct * rho * n_rps^2) )^(1/4); 

% Calculate disc area S for Froude efficiency
S = pi * (D_prop / 2)^2; %from AI

% Eq (6): Useful power consumed by the propeller in hover mode
P_prop = M_prop * ((2 * pi * n_rpm) / 60); 

% Eq (4): Ideal relative Froude aerodynamic efficiency (eta_0)
eta_0 = sqrt( T_prop^3 / (2 * rho * S) ) / P_prop;

%% 3. Motor Selection Calculations (Section II-B)

% Eq (9): Motor torque coefficient K_t [N*m/A] from Kv
Kt = 1 / ((Kv_rpm_v * 2 * pi) / 60); 

% Eq (8): Maximum electromagnetic torque of the motor
M_mot = Kt * I_max; 

% Motor power estimation (assuming matching torque at target RPM for electrical power input)
omega = (2 * pi * n_rpm) / 60;
P_mot = M_mot * omega; % Maximum electrical power capacity benchmark

% Eq (5): Thrust-to-motor-power ratio indicator
thrust_to_power_ratio = T_prop / P_mot; 

% Eq (7): Propeller-to-motor power efficiency metric
eta = P_prop / P_mot;

%% 4. ESC Selection Calculations (Section II-C)
% ESC sizing rule: Must be strictly greater than or equal to motor continuous max current
I_ESC_cont = 1.2 * I_mot_cont; % Applying standard engineering 20% safety margin

%% 5. Display Formatted Results
fprintf('==================================================\n');
fprintf('     Multirotor Propulsion Design Parameters     \n');
fprintf('==================================================\n');
fprintf('Required Aircraft Total Thrust (T_AC) : %.2f N\n', T_AC);
fprintf('Thrust per Single Propeller (T_prop)   : %.2f N\n', T_prop);
fprintf('Calculated Propeller Diameter (D_prop) : %.2f inches (%.3f m)\n', D_prop * 39.3701, D_prop);
fprintf('Propeller Swept Area (S)               : %.4f m^2\n', S);
fprintf('Useful Propeller Hover Power (P_prop)  : %.2f W\n', P_prop);
fprintf('Ideal Froude Efficiency (eta_0)        : %.2f%%\n', eta_0 * 100);
fprintf('--------------------------------------------------\n');
fprintf('Motor Torque Coefficient (K_t)         : %.4f N*m/A\n', Kt);
fprintf('Max Motor Electromagnetic Torque(M_mot): %.4f N*m\n', M_mot);
fprintf('Overall System Efficiency Metric (eta) : %.2f%%\n', eta * 100);
fprintf('--------------------------------------------------\n');
fprintf('Motor Max Continuous Current (I_mot)  : %.1f A\n', I_mot_cont);
fprintf('Recommended ESC Continuous Rating     : %.1f A\n', I_ESC_cont);

% Mechanical safety check mentioned in Section II-B text
if M_mot >= M_prop
    fprintf('STATUS: [PASS] Motor torque exceeds propeller torque load.\n');
else
    fprintf('STATUS: [FAIL] Motor torque is insufficient for this propeller load!\n');
end

% ESC inequality validation from Section II-C
if I_ESC_cont >= I_mot_cont
    fprintf('STATUS: [PASS] ESC current requirement satisfied.\n');
else
    fprintf('STATUS: [FAIL] ESC capacity is below continuous motor current!\n');
end
fprintf('==================================================\n');
