clear; clc; close all;

g = 9.81;
m_empty = 1.8;            % Aircraft mass INCLUDING battery (kg)
m_payload = 0.00;          % Payload mass (kg)
N_rotors=4;

m_total = m_empty + m_payload;
W = m_total * g;           % Total aircraft weight (N)


battery_cells = 4;         % Number of LiPo cells
battery_capacity_Ah = 6.5; % Battery capacity (Ah)
battery_voltage = 14.8;    % Nominal voltage (V)
battery_max_C = 60;        % Battery discharge rating (C)
battery_weight = NaN;      % Battery weight (kg)

usable_fraction = 0.80;    % Assumed usable fraction (replace with justified value)
E_nominal = battery_voltage * battery_capacity_Ah;
E_usable = E_nominal * usable_fraction; % Wh


% T_max_motor = max(manufacturer_thrust_N);
% T_max_total = N_rotors * T_max_motor; 
% TWR = T_max_total / W;
% T_design_motor = ...
%     (TWR_required * W) / N_rotors; 
% disk_loading = W / A_total; % N/m^2

T_hover_motor = W / N_rotors;


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

% manufacturer_RPM = [ ...
%     3000 3800 4500 5100 5600 6100 6500];

% Convert thrust from kgf to Newton
manufacturer_thrust_N = manufacturer_thrust_kgf * g;

% Calculate electrical power for one motor
manufacturer_power_W = ...
    manufacturer_voltage_V .* manufacturer_current_A;

% Thrust per electrical watt
manufacturer_thrust_per_W = ...
    manufacturer_thrust_N ./ manufacturer_power_W;

%% =========================================================
%  9. HOVER CURRENT AND POWER ESTIMATION
% ==========================================================

% Sort manufacturer data by thrust
[T_sorted,idx] = sort(manufacturer_thrust_N)

I_sorted = manufacturer_current_A(idx)
V_sorted = manufacturer_voltage_V(idx)
P_sorted = manufacturer_power_W(idx)

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
    % vq = interp1(x, v, xq, 'linear') ini format nya
    I_hover = interp1(T_sorted,I_sorted,T_hover_motor,'linear');
    V_hover = interp1(T_sorted,V_sorted,T_hover_motor,'linear');

    P_hover_motor = V_hover * I_hover;

    % Total electrical power for all motors + avionics
    % P_total_hover = N_rotors * P_hover_motor + avionics_power;
    P_total_hover = N_rotors * P_hover_motor;

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
