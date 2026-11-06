%% MULTIROTOR PROPULSION SYSTEM EVALUATION
% Evaluates:
% 1. Motor and propeller specifications
% 2. Manufacturer thrust-test data
% 3. Theoretical propeller thrust and torque
% 4. Aircraft thrust-to-weight ratio
% 5. Hover power and endurance
% 6. Payload versus endurance
% 7. Propulsion performance plots
%
% IMPORTANT:
% Manufacturer data must be for ONE motor + ONE propeller.
% Replace all illustrative data with actual specifications.

clear; clc; close all;

%% =========================================================
%  1. AIRCRAFT INPUTS
% ==========================================================

N_rotors = 4;              % Number of motors
g = 9.80665;               % Gravity (m/s^2)
rho = 1.225;               % Air density (kg/m^3)

m_empty = 1.70;            % Aircraft mass INCLUDING battery (kg)
m_payload = 0.00;          % Payload mass (kg)

m_total = m_empty + m_payload;
W = m_total * g;           % Total aircraft weight (N)

TWR_required = 2.0;        % Desired design TWR

%% =========================================================
%  2. MOTOR SPECIFICATIONS
% ==========================================================

motor_KV = 880;            % Motor KV (RPM/V)
motor_max_current = 20;    % Maximum continuous motor current (A)
motor_max_power = NaN;     % Manufacturer maximum power (W)
motor_weight = NaN;        % Motor weight (kg)

%% =========================================================
%  3. PROPELLER SPECIFICATIONS
% ==========================================================

prop_diameter_in = 9;     % Propeller diameter (inch)
prop_pitch_in = 4.5;       % Propeller pitch (inch)
prop_blades = 2;           % Number of blades
prop_weight = NaN;         % Propeller weight (kg)

D = prop_diameter_in * 0.0254;  % Diameter (m)
P = prop_pitch_in * 0.0254;     % Pitch (m)

A_prop = pi * (D/2)^2;          % One propeller disk area (m^2)
A_total = N_rotors * A_prop;    % Total disk area (m^2)

%% =========================================================
%  4. ESC SPECIFICATIONS
% ==========================================================

ESC_max_current = 30;      % Continuous ESC current rating (A)
ESC_weight = NaN;          % ESC weight (kg)

%% =========================================================
%  5. BATTERY SPECIFICATIONS
% ==========================================================

battery_cells = 4;         % Number of LiPo cells
battery_capacity_Ah = 6.5; % Battery capacity (Ah)
battery_voltage = 14.8;    % Nominal voltage (V)
battery_max_C = 60;        % Battery discharge rating (C)
battery_weight = NaN;      % Battery weight (kg)

usable_fraction = 0.80;    % Assumed usable fraction (replace with justified value)
avionics_power = 10;       % Flight controller, RPi, camera, etc. (W)

E_nominal = battery_voltage * battery_capacity_Ah;
E_usable = E_nominal * usable_fraction; % Wh

%% =========================================================
%  6. MANUFACTURER MOTOR-PROPELLER TEST DATA
% ==========================================================
% DATA MUST REPRESENT ONE MOTOR + ONE PROPELLER.
%
% Columns:
% Thrust: kgf
% Current: A
% Voltage: V
% RPM: rpm
%
% Replace these illustrative values with manufacturer data.
% If RPM is unavailable, use NaN for the entire RPM column.

manufacturer_thrust_kgf = [ ...
    0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 0.95];

manufacturer_current_A = [ ...
    0.7 1.6 2.7 4.0 5.6 6.9 8.6 10.1 12.2 13.0];

manufacturer_voltage_V = [ ...
    14.8 14.8 14.8 14.8 14.8 14.8 14.8 14.8 14.8 14.8];

manufacturer_RPM = [ ...
    1000 2000 3000 4000 5000 6000 7000 8000 9000 10000];

% manufacturer_RPM = [ ...
%     NaN NaN NaN NaN NaN NaN NaN NaN NaN NaN];

% Convert thrust from kgf to Newton
manufacturer_thrust_N = manufacturer_thrust_kgf * g;

% Calculate electrical power for one motor
manufacturer_power_W = ...
    manufacturer_voltage_V .* manufacturer_current_A;

% Thrust per electrical watt
manufacturer_thrust_per_W = ...
    manufacturer_thrust_N ./ manufacturer_power_W;

%% =========================================================
%  7. THEORETICAL PROPELLER COEFFICIENTS
% ==========================================================
% Set these to actual propeller coefficients if available.
% Do not assume universal values for different propellers.
%
% If unknown, leave as NaN and theoretical calculation is skipped.

CT0 = NaN;   % Static thrust coefficient
CP0 = NaN;   % Static power coefficient

% Example only:
% CT0 = 0.11;
% CP0 = 0.05;
%
% These example coefficients are NOT verified for your propeller.

%% =========================================================
%  8. BASIC AIRCRAFT PERFORMANCE
% ==========================================================

T_max_motor = max(manufacturer_thrust_N);
T_max_total = N_rotors * T_max_motor;

TWR = T_max_total / W;

T_hover_motor = W / N_rotors;

T_design_motor = ...
    (TWR_required * W) / N_rotors;

disk_loading = W / A_total; % N/m^2

fprintf('\n========================================\n');
fprintf(' AIRCRAFT PROPULSION PERFORMANCE\n');
fprintf('========================================\n');

fprintf('Total aircraft mass       : %.3f kg\n',m_total);
fprintf('Total aircraft weight     : %.2f N\n',W);
fprintf('Number of rotors          : %d\n',N_rotors);
fprintf('Maximum thrust per motor  : %.2f N\n',T_max_motor);
fprintf('Maximum total thrust      : %.2f N\n',T_max_total);
fprintf('Thrust-to-weight ratio    : %.2f\n',TWR);
fprintf('Required hover thrust     : %.2f N/motor\n',T_hover_motor);
fprintf('Design thrust requirement : %.2f N/motor\n',T_design_motor);
fprintf('Disk loading              : %.2f N/m^2\n',disk_loading);

if TWR >= TWR_required
    fprintf('TWR requirement            : PASSED\n');
else
    fprintf('TWR requirement            : NOT PASSED\n');
end

%% =========================================================
%  9. HOVER CURRENT AND POWER ESTIMATION
% ==========================================================

% Sort manufacturer data by thrust
[T_sorted,idx] = sort(manufacturer_thrust_N);

I_sorted = manufacturer_current_A(idx);
V_sorted = manufacturer_voltage_V(idx);
P_sorted = manufacturer_power_W(idx);

% Check whether hover thrust is within tested range
if T_hover_motor < min(T_sorted) || T_hover_motor > max(T_sorted)

    warning(['Hover thrust is outside manufacturer test range. ', ...
             'Hover current and endurance cannot be reliably interpolated.']);

    I_hover = NaN;
    V_hover = NaN;
    P_hover_motor = NaN;
    P_total_hover = NaN;
    endurance_hover_min = NaN;

else

    % Interpolate manufacturer data at required hover thrust
    I_hover = interp1(T_sorted,I_sorted,T_hover_motor,'linear');
    V_hover = interp1(T_sorted,V_sorted,T_hover_motor,'linear');

    P_hover_motor = V_hover * I_hover;

    % Total electrical power for all motors + avionics
    P_total_hover = N_rotors * P_hover_motor + avionics_power;

    % Battery endurance
    endurance_hover_min = ...
        (E_usable / P_total_hover) * 60;

end

fprintf('\n========================================\n');
fprintf(' HOVER PERFORMANCE\n');
fprintf('========================================\n');

fprintf('Hover thrust per motor : %.2f N\n',T_hover_motor);
fprintf('Hover current per motor: %.2f A\n',I_hover);
fprintf('Hover voltage          : %.2f V\n',V_hover);
fprintf('Hover power per motor  : %.2f W\n',P_hover_motor);
fprintf('Total hover power      : %.2f W\n',P_total_hover);
fprintf('Usable battery energy  : %.2f Wh\n',E_usable);
fprintf('Estimated endurance   : %.2f min\n',endurance_hover_min);

%% =========================================================
%  10. ELECTRICAL AND THERMAL LIMIT CHECKS
% ==========================================================

fprintf('\n========================================\n');
fprintf(' COMPONENT LIMIT CHECKS\n');
fprintf('========================================\n');

if ~isnan(I_hover)
    fprintf('Motor current limit: ');
    if I_hover <= motor_max_current
        fprintf('PASSED\n');
    else
        fprintf('EXCEEDED\n');
    end

    fprintf('ESC current limit  : ');
    if I_hover <= ESC_max_current
        fprintf('PASSED\n');
    else
        fprintf('EXCEEDED\n');
    end
end

% Battery maximum theoretical current
battery_max_current = battery_capacity_Ah * battery_max_C;

fprintf('Battery max current (C-rating): %.2f A\n', ...
    battery_max_current);

if ~isnan(I_hover)
    total_hover_current = N_rotors * I_hover;

    fprintf('Estimated total motor current: %.2f A\n', ...
        total_hover_current);

    if total_hover_current <= battery_max_current
        fprintf('Battery current limit: PASSED\n');
    else
        fprintf('Battery current limit: EXCEEDED\n');
    end
end

%% =========================================================
%  11. THEORETICAL THRUST AND TORQUE
% ==========================================================
% Based on static propeller equations:
%
% T = CT0 * rho * n^2 * D^4
%
% Q = CP0 * rho * n^2 * D^5 / (2*pi)
%
% n = RPM/60 (revolutions per second)
%
% These equations do not explicitly contain pitch.
% Pitch and blade geometry influence CT0 and CP0.

if ~isnan(CT0) && ~isnan(CP0) && ...
        all(~isnan(manufacturer_RPM))

    n_rps = manufacturer_RPM / 60;

    T_theoretical = ...
        CT0 * rho .* n_rps.^2 * D^4;

    Q_theoretical = ...
        CP0 * rho .* n_rps.^2 * D^5 / (2*pi);

    omega = 2*pi*n_rps;

    P_shaft_theoretical = Q_theoretical .* omega;

    theoretical_results = table( ...
        manufacturer_RPM(:), ...
        T_theoretical(:), ...
        Q_theoretical(:), ...
        P_shaft_theoretical(:), ...
        'VariableNames', ...
        {'RPM','Thrust_N','Torque_Nm','ShaftPower_W'});

    fprintf('\nTHEORETICAL PROPELLER RESULTS\n');
    disp(theoretical_results);

else
    fprintf('\nTheoretical propeller calculation skipped.\n');
    fprintf('Provide valid CT0, CP0, and RPM data.\n');
end

%% =========================================================
%  12. PAYLOAD VS ENDURANCE
% ==========================================================
% Assumes the same battery and propeller configuration.
% Uses manufacturer data to estimate power at each hover thrust.
% Does not include changes in battery voltage under load.

payload_range = linspace(0,2,21); % kg; EDIT to suit aircraft

endurance_payload = NaN(size(payload_range));
TWR_payload = NaN(size(payload_range));

for k = 1:length(payload_range)

    current_mass = m_empty + payload_range(k);
    current_weight = current_mass * g;

    required_thrust = current_weight / N_rotors;

    TWR_payload(k) = T_max_total / current_weight;

    if required_thrust >= min(T_sorted) && ...
            required_thrust <= max(T_sorted)

        current_I = interp1( ...
            T_sorted,I_sorted,required_thrust,'linear');

        current_V = interp1( ...
            T_sorted,V_sorted,required_thrust,'linear');

        current_power = ...
            N_rotors * current_I * current_V + avionics_power;

        endurance_payload(k) = ...
            (E_usable / current_power) * 60;
    end
end

%% =========================================================
%  13. RESULTS TABLE
% ==========================================================

results_table = table( ...
    manufacturer_thrust_N(:), ...
    manufacturer_RPM(:), ...
    manufacturer_current_A(:), ...
    manufacturer_voltage_V(:), ...
    manufacturer_power_W(:), ...
    manufacturer_thrust_per_W(:), ...
    'VariableNames', ...
    {'Thrust_N','RPM','Current_A','Voltage_V', ...
     'ElectricalPower_W','ThrustPerWatt_N_per_W'});

fprintf('\n========================================\n');
fprintf(' MANUFACTURER PERFORMANCE DATA\n');
fprintf('========================================\n');

disp(results_table);

%% =========================================================
%  14. PERFORMANCE PLOTS
% ==========================================================

% Plot 1: Thrust vs RPM
if all(~isnan(manufacturer_RPM))
    figure;
    plot(manufacturer_RPM,manufacturer_thrust_N,'o-','LineWidth',1.5);
    grid on;
    xlabel('Motor speed (RPM)');
    ylabel('Thrust per motor (N)');
    title('Static Thrust vs Motor RPM');
end

% Plot 2: Current vs Thrust
figure;
plot(manufacturer_thrust_N,manufacturer_current_A,...
    'o-','LineWidth',1.5);
hold on;
if ~isnan(I_hover)
    xline(T_hover_motor,'--r','Hover thrust');
end
grid on;
xlabel('Thrust per motor (N)');
ylabel('Current (A)');
title('Motor Current vs Thrust');

% Plot 3: Electrical power vs Thrust
figure;
plot(manufacturer_thrust_N,manufacturer_power_W,...
    'o-','LineWidth',1.5);
hold on;
if ~isnan(P_hover_motor)
    xline(T_hover_motor,'--r','Hover thrust');
end
grid on;
xlabel('Thrust per motor (N)');
ylabel('Electrical power per motor (W)');
title('Electrical Power vs Thrust');

% Plot 4: Thrust per watt
figure;
plot(manufacturer_thrust_N,manufacturer_thrust_per_W,...
    'o-','LineWidth',1.5);
grid on;
xlabel('Thrust per motor (N)');
ylabel('Thrust / Electrical Power (N/W)');
title('Static Thrust-to-Power Ratio');

% Plot 5: Endurance vs payload
figure;
plot(payload_range,endurance_payload,...
    'o-','LineWidth',1.5);
grid on;
xlabel('Payload mass (kg)');
ylabel('Estimated hover endurance (min)');
title('Estimated Hover Endurance vs Payload');

% Plot 6: TWR vs payload
figure;
plot(payload_range,TWR_payload,...
    'o-','LineWidth',1.5);
hold on;
yline(TWR_required,'--r','Required TWR');
grid on;
xlabel('Payload mass (kg)');
ylabel('Thrust-to-weight ratio');
title('Thrust-to-Weight Ratio vs Payload');

%% =========================================================
%  15. SUMMARY
% ==========================================================

fprintf('\n========================================\n');
fprintf(' FINAL PROPULSION SUMMARY\n');
fprintf('========================================\n');

fprintf('Motor KV              : %.0f RPM/V\n',motor_KV);
fprintf('Propeller             : %.1f x %.1f inch\n', ...
    prop_diameter_in,prop_pitch_in);
fprintf('Battery               : %.1f V, %.1f Ah\n', ...
    battery_voltage,battery_capacity_Ah);
fprintf('Total aircraft mass   : %.2f kg\n',m_total);
fprintf('Maximum total thrust  : %.2f N\n',T_max_total);
fprintf('TWR                   : %.2f\n',TWR);

if ~isnan(endurance_hover_min)
    fprintf('Estimated hover time : %.2f minutes\n', ...
        endurance_hover_min);
else
    fprintf('Estimated hover time : Not available\n');
end

fprintf('========================================\n');
