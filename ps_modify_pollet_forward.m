%% MULTIROTOR HOVER EVALUATION (MATLAB equivalent of calculate_hover.py)
% Requires MATLAB with table support (no additional toolboxes).
% APC-based static coefficients; battery and electrical inputs are illustrative.
clear; clc; close all;

g = 9.80665;
rho = 1.3;
N_rotors = 4;
m_without_battery = 1.20;  % kg, ASSUMED
m_payload = 0.0;           % kg
avionics_power = 10;       % W, ASSUMED
usable_fraction = 0.80;    % ASSUMED
eta_ESC_motor = 0.75;      % combined efficiency, ASSUMED
motor_KV = 880;            % RPM/V
ESC_current_limit = 30;    % A, assumed battery-side per-rotor limit

r_prime = 0.75;
sigma_e = 0.0707; % not in pollet

% T_hover_per_rotor = W/N_rotors;
% T_design_per_rotor = TWR_required*W/N_rotors;

% variabel
V_inf = 1:1:10;       
alpha = deg2rad(1:4:45); % Incidence angle (rad)
propellers = [9 4.5; 9 4.7; 10 4.5; 10 4.7];
RPM_values = 3000:500:9000;
batteries = [4 5.2 0.50 30;
             4 6.5 0.60 30;
             6 5.0 0.72 30];
battery_names = {'4S_5200', '4S_6500', '6S_5000'};

%% 3. Calculate all combinations
all_rows = struct([]);
matched_rows = struct([]);

for i = 1:size(propellers,1)

    diameter_in = propellers(i,1);
    pitch_in = propellers(i,2);
    D = diameter_in * 0.0254;
    beta = pitch_in / diameter_in;
    beta_prime = atan(pitch_in / (pi * diameter_in * r_prime)); % not in pollet
    % Pollet et al., equations 8a/8b, static J_axial = 0
    CT0 = 0.02791 + 0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3;
    CP0 = 0.01813 - 0.06218*beta + 0.35712*beta^2 - 0.23774*beta^3;
    % assert(CT0 > 0 && CP0 > 0, 'Invalid surrogate coefficients');


    CT_axial = 0.02791 - 0.06543*J_axial + 0.11867*beta ...
                + 0.27334*beta^2 - 0.28852*beta^3 ...
                + 0.02104*J_axial^3 - 0.23504*J_axial^2 + 0.18677*beta^2*J_axial;
    CP_axial = 0.01813 - 0.06218*beta + 0.00343*J_axial ...
        + 0.35712*beta^2 - 0.23774*beta^3 ...
        + 0.07549*beta*J_axial - 0.1235*J_axial^2;

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

    for j = 1:size(batteries,1)

        S = batteries(j,1);
        Ah = batteries(j,2);
        battery_mass = batteries(j,3);
        C_rating = batteries(j,4);
        battery_name = battery_names{j};

        m_total = m_without_battery + m_payload + battery_mass;
        T_hover = m_total * g / N_rotors;
        V_battery = 3.7 * S;   % nominal voltage
        E_usable = V_battery * Ah * usable_fraction; % Wh
        I_battery_max = Ah * C_rating;
        RPM_no_load = motor_KV * V_battery; % ideal ceiling, NOT motor validation

        % Same columns and calculation as Python 'common' fields
        common = struct('Propeller',sprintf('%gx%g',diameter_in,pitch_in), ...
            'Diameter_in',diameter_in,'Pitch_in',pitch_in, ...
            'Battery',battery_name,'Battery_S',S,'Battery_Ah',Ah, ...
            'BatteryMass_kg',battery_mass,'TotalMass_kg',m_total, ...
            'NominalVoltage_V',V_battery, ...
            'RequiredHoverThrustPerRotor_N',T_hover, ...
            'CT_static',CT0,'CP_static',CP0, ...
            'UsableBatteryEnergy_Wh',E_usable);

        % All sampled operating points
        for k = 1:numel(RPM_values)
            RPM = RPM_values(k);
            n = RPM / 60;

            for x = 1:numel(V_inf)
                for y = 1:numel(alpha)
                J = V_inf(x) / (n * D);
                J_axial = J * sin(alpha(y));
                
                CT_axial = 0.02791 - 0.06543*J_axial + 0.11867*beta ...
                    + 0.27334*beta^2 - 0.28852*beta^3 ...
                    + 0.02104*J_axial^3 - 0.23504*J_axial^2 + 0.18677*beta^2*J_axial;
                CP_axial = 0.01813 - 0.06218*beta + 0.00343*J_axial ...
                    + 0.35712*beta^2 - 0.23774*beta^3 ...
                    + 0.07549*beta*J_axial - 0.1235*J_axial^2;

                delta = (3/2) * cos(beta_prime) * ...
                    (1 + (sigma_e / tan(beta_prime)) * ...
                    (1 + sqrt(1 + 2*tan(beta_prime)/sigma_e)) * ...
                    (1 - cos(alpha(y))));
                numerator = (J * cos(alpha(y)) / (pi*r_prime))^2;
                
                eta_T = 1 + numerator * delta / ...
                    (2 * (1 - J_axial / J0T));
                eta_P = 1 + numerator * delta / ...
                    (2 * (1 - J_axial / J0P));

                CT_corrected = eta_T * CT_axial;
                CP_corrected = eta_P * CP_axial;

                Thrust = CT_corrected * rho * n^2 * D^4;
                Pshaft = CP_corrected * rho * n^3 * D^5;
                Torque = Pshaft / (2*pi*n);
                Pelec_rotor = Pshaft / eta_ESC_motor;
                Pelec_total = N_rotors*Pelec_rotor + avionics_power;
                I_rotor = Pelec_rotor / V_battery;
                I_battery = N_rotors*I_rotor + avionics_power/V_battery;
                endurance_min = E_usable/Pelec_total*60;
    
                row = common;
                row.RPM = RPM;
                row.V_inf = V_inf(x);
                row.Alpha_deg = rad2deg(alpha(y));
                row.J = J;
                row.J_axial = J_axial;
                row.CT_corrected = CT_corrected;
                row.CP_corrected = CP_corrected;
                row.eta_T = eta_T;
                row.eta_P = eta_P;
                row.ThrustPerRotor_N = Thrust;
                row.ShaftPowerPerRotor_W = Pshaft;
                row.TorquePerRotor_Nm = Torque;
                row.EstimatedElectricalPowerPerRotor_W = Pelec_rotor;
                row.EstimatedTotalElectricalPower_W = Pelec_total;
                row.EstimatedBatteryCurrent_A = I_battery;
                row.EstimatedEndurance_min = endurance_min;
                row.EstimatedDCInputPerRotor_A = I_rotor;
                
                row.EnoughThrust = Thrust >= T_hover;
                row.BelowIdealNoLoadRPM = RPM < RPM_no_load;
                row.WithinAssumedESCDCInputLimit = I_rotor <= ESC_current_limit;
                row.WithinRatedBatteryC = I_battery <= I_battery_max;
                
                if isempty(all_rows)
                    all_rows = row;
                else
                    all_rows(end+1) = row;
                end
                end
            end
        end

        % 
        % % Match RPM to exact required hover thrust (fair comparison)
        % RPM_hover = 60 * sqrt(T_hover / (CT0*rho*D^4));
        % n = RPM_hover / 60;
        % Thrust = CT_corrected * rho * n^2 * D^4;
        % Pshaft = CP_corrected * rho * n^3 * D^5;
        % Torque = Pshaft / (2*pi*n);
        % Pelec_rotor = Pshaft / eta_ESC_motor;
        % Pelec_total = N_rotors*Pelec_rotor + avionics_power;
        % I_rotor = Pelec_rotor / V_battery;
        % I_battery = N_rotors*I_rotor + avionics_power/V_battery;
        % endurance_min = E_usable/Pelec_total*60;
        % 
        % row = common;
        % row.RPM = RPM;
        % row.V_inf = V_inf(x);
        % row.Alpha_deg = rad2deg(alpha(y));
        % row.J = J;
        % row.J_axial = J_axial;
        % row.CT_corrected = CT_corrected;
        % row.CP_corrected = CP_corrected;
        % row.eta_T = eta_T;
        % row.eta_P = eta_P;
        % row.ThrustPerRotor_N = Thrust;
        % row.ShaftPowerPerRotor_W = Pshaft;
        % row.TorquePerRotor_Nm = Torque;
        % row.EstimatedElectricalPowerPerRotor_W = Pelec_rotor;
        % row.EstimatedTotalElectricalPower_W = Pelec_total;
        % row.EstimatedBatteryCurrent_A = I_battery;
        % row.EstimatedEndurance_min = endurance_min;
        % row.EstimatedDCInputPerRotor_A = I_rotor;
        % 
        % row.PassesBasicElectricalScreen = RPM_hover < RPM_no_load && ...
        %     I_rotor <= ESC_current_limit && I_battery <= I_battery_max;
        % row.BelowIdealNoLoadRPM = RPM_hover < RPM_no_load;
        % row.WithinAssumedESCDCInputLimit = I_rotor <= ESC_current_limit;
        % row.WithinRatedBatteryC = I_battery <= I_battery_max;
        % if isempty(matched_rows)
        %     matched_rows = row;
        % else
        %     matched_rows(end+1) = row;
        % end
    end
end

%% 4. Save and display
HoverAll = struct2table(all_rows);
size(HoverAll)
% HoverMatched = struct2table(matched_rows);
% size(HoverMatched)
writetable(HoverAll,'hover_all_points_matlab.csv');
% writetable(HoverMatched,'hover_matched_matlab.csv');

% fprintf('NOTE: APC-derived coefficients and assumed masses, efficiency, limits.\n');
% fprintf('Not validated Gemfan/SunnySky performance predictions.\n');
fprintf('kelar\n');
