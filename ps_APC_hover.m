%% MULTIROTOR HOVER EVALUATION (MATLAB equivalent of calculate_hover.py)
% Requires MATLAB with table support (no additional toolboxes).
% APC-based static coefficients; battery and electrical inputs are illustrative.
clear; clc; close all;

%% 1. Aircraft and assumptions
g = 9.80665;
rho = 1.225;  % kg/m^3, ISA sea-level standard;
N_rotors = 4;
m_without_battery = 1.20;  % kg, ASSUMED
m_payload = 0.0;           % kg
avionics_power = 20;       % W, ASSUMED
usable_fraction = 0.750;    % ASSUMED
eta_ESC_motor = 0.80;      % combined efficiency, ASSUMED
motor_KV = 880;            % RPM/V
ESC_current_limit = 30;    % A, assumed battery-side per-rotor limit

%% 2. Candidate inputs
% Each row is one REAL propeller candidate: [diameter_in, pitch_in]
propellers = [9 4.5; 9 4.7; 10 4.5; 10 4.7];
prop_name = {'9x4.5MR','9x4.7SF','10x4.5MR','10x4.7SF'};
RPM_values = 4000:500:9000;

% [S, Capacity_Ah, Mass_kg, C_rating]
batteries = [
    4  5.2   0.490  50;
    4  6.5   0.550  60;
    4  7.2   0.630  50;
    4 10.0   0.890  25;
    6  2.6   0.410  50;
    6  3.7   0.590  50;
    6  4.5   0.715  65;
    6  5.2   0.74   50;
    6  8.0   1.100  25
    ];

battery_names = {
    '4S_5200', '4S_6500', '4S_7200', '4S_10000', ...
    '6S_2600', '6S_3700', '6S_4500', '6S_5200', ...
    '6S_8000'
    };

% 4S_5200: https://tk.tokopedia.com/ZSpBg1uEM/
% 4s_7200:https://tk.tokopedia.com/ZSpBpSudu/
% 6S_2600: https://tk.tokopedia.com/ZSpBg5Qut/
% 6S_3700: https://tk.tokopedia.com/ZSpBg8XM9/
% 6S_4500: https://tk.tokopedia.com/ZSpBqwBtm/
% 6S_5200: https://tk.tokopedia.com/ZSpBsSPF9/
% 6S_8000: https://tk.tokopedia.com/ZSpBqneJg/

%% 3. Calculate all combinations
all_rows = struct([]);
matched_rows = struct([]);

for i = 1:size(propellers,1)

    diameter_in = propellers(i,1);
    pitch_in = propellers(i,2);
    D = diameter_in * 0.0254;
    % beta = pitch_in / diameter_in;

    % Pollet et al., equations 8a/8b, static J_axial = 0
    % CT0 = 0.02791 + 0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3;
    % CP0 = 0.01813 - 0.06218*beta + 0.35712*beta^2 - 0.23774*beta^3;
    % assert(CT0 > 0 && CP0 > 0, 'Invalid surrogate coefficients');

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

        % All sampled operating points
        for k = 1:numel(RPM_values)
            RPM = RPM_values(k);

            data_prop = apcProp(prop_name{i}, RPM, 0);
            if ~isfinite(data_prop.Ct) || ~isfinite(data_prop.Cp) ...
                    || data_prop.Ct <= 0 || data_prop.Cp <= 0
                continue;
            end
            CP0 = data_prop.Cp;
            CT0 = data_prop.Ct;

            % Same columns and calculation as Python 'common' fields
            common = struct('Propeller',sprintf('%gx%g',diameter_in,pitch_in), ...
                'Diameter_in',diameter_in,'Pitch_in',pitch_in, ...
                'Battery',battery_name,'Battery_S',S,'Battery_Ah',Ah, ...
                'BatteryMass_kg',battery_mass,'TotalMass_kg',m_total, ...
                'NominalVoltage_V',V_battery, ...
                'RequiredHoverThrustPerRotor_N',T_hover, ...
                'UsableBatteryEnergy_Wh',E_usable);

            n = RPM / 60;
            Thrust = CT0 * rho * n^2 * D^4;
            Pshaft = CP0 * rho * n^3 * D^5;
            Torque = Pshaft / (2*pi*n);
            Pelec_rotor = Pshaft / eta_ESC_motor;
            Pelec_total = N_rotors*Pelec_rotor + avionics_power;
            I_rotor = Pelec_rotor / V_battery;
            I_battery = N_rotors*I_rotor + avionics_power/V_battery;
            endurance_min = E_usable/Pelec_total*60;

            row = common;
            row.RPM = RPM;
            row.ThrustPerRotor_N = Thrust;
            row.ShaftPowerPerRotor_W = Pshaft;
            row.TorquePerRotor_Nm = Torque;
            row.EstimatedElectricalPowerPerRotor_W = Pelec_rotor;
            row.EstimatedTotalElectricalPower_W = Pelec_total;
            row.CT_APC = CT0;
            row.CP_APC = CP0;
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


        %% MATCH RPM TO REQUIRED HOVER THRUST

        % Calculate thrust at the endpoints of the RPM range
        RPM_min = min(RPM_values);
        RPM_max = max(RPM_values);

        data_min = apcProp(prop_name{i}, RPM_min, 0);
        data_max = apcProp(prop_name{i}, RPM_max, 0);

        T_min = data_min.Ct*rho*(RPM_min/60)^2*D^4;
        T_max = data_max.Ct*rho*(RPM_max/60)^2*D^4;

        % Check whether target thrust is bracketed
        if ~isfinite(T_min) || ~isfinite(T_max) || ...
                (T_min-T_hover)*(T_max-T_hover) > 0
            fprintf('No hover RPM bracket: %s, %s\n', ...
                prop_name{i}, battery_name);
            continue;
        end

        % Special case: target equals an endpoint
        if T_min == T_hover
            RPM_hover = RPM_min;
        elseif T_max == T_hover
            RPM_hover = RPM_max;
        else
            % Solve for RPM using APC CT interpolation
            thrust_error = @(rpm) apcHoverThrust(rpm,prop_name{i},rho,D) - T_hover;

            RPM_hover = fzero(thrust_error,[RPM_min RPM_max]);
        end

        %% Evaluate CT and CP at matched RPM
        data_prop_hover = apcProp(prop_name{i},RPM_hover,0);

        CT0_hover = data_prop_hover.Ct;
        CP0_hover = data_prop_hover.Cp;

        n = RPM_hover/60;

        Thrust = CT0_hover*rho*n^2*D^4;
        Pshaft = CP0_hover*rho*n^3*D^5;
        Torque = Pshaft/(2*pi*n);

        Pelec_rotor = Pshaft/eta_ESC_motor;
        Pelec_total = N_rotors*Pelec_rotor + avionics_power;

        I_rotor = Pelec_rotor/V_battery;
        I_battery = Pelec_total/V_battery;

        endurance_min = E_usable/Pelec_total*60;

        %% Save matched data
        row = common;
        row.CT_APC_hover= CT0_hover;
        row.CP_APC_hover = CP0_hover;

        row.RequiredHoverRPM = RPM_hover;
        row.ThrustPerRotor_N = Thrust;
        row.ThrustError_N = Thrust-T_hover;
        row.ShaftPowerPerRotor_W = Pshaft;
        row.TorquePerRotor_Nm = Torque;
        row.EstimatedElectricalPowerPerRotor_W = Pelec_rotor;
        row.EstimatedTotalElectricalPower_W = Pelec_total;
        row.EstimatedBatteryCurrent_A = I_battery;
        row.EstimatedEndurance_min = endurance_min;
        row.EstimatedDCInputPerRotor_A = I_rotor;

        row.BelowIdealNoLoadRPM = RPM_hover < RPM_no_load;
        row.WithinAssumedESCDCInputLimit = ...
            I_rotor <= ESC_current_limit;
        row.WithinRatedBatteryC = ...
            I_battery <= I_battery_max;

        row.PassesBasicElectricalScreen = ...
            row.BelowIdealNoLoadRPM && ...
            row.WithinAssumedESCDCInputLimit && ...
            row.WithinRatedBatteryC;

        if isempty(matched_rows)
            matched_rows = row;
        else
            matched_rows(end+1) = row;
        end
        % 
        % % Match RPM to exact required hover thrust (fair comparison)
        % RPM_hover = 60 * sqrt(T_hover / (CT0*rho*D^4));
        % n = RPM_hover / 60;
        % data_prop_hover = apcProp(prop_name{i}, RPM_hover, 0);
        % 
        % CP0_hover = data_prop_hover.Cp;
        % CT0_hover = data_prop_hover.Ct;
        % 
        % Thrust = CT0_hover * rho * n^2 * D^4;
        % Pshaft = CP0_hover * rho * n^3 * D^5;
        % Torque = Pshaft / (2*pi*n);
        % Pelec_rotor = Pshaft / eta_ESC_motor;
        % Pelec_total = N_rotors*Pelec_rotor + avionics_power;
        % I_rotor = Pelec_rotor / V_battery;
        % I_battery = N_rotors*I_rotor + avionics_power/V_battery;
        % endurance_min = E_usable/Pelec_total*60;
        % 
        % row = common;
        % row.RequiredHoverRPM = RPM_hover;
        % row.ThrustPerRotor_N = Thrust;
        % row.ShaftPowerPerRotor_W = Pshaft;
        % row.TorquePerRotor_Nm = Torque;
        % row.EstimatedElectricalPowerPerRotor_W = Pelec_rotor;
        % row.EstimatedTotalElectricalPower_W = Pelec_total;
        % row.EstimatedBatteryCurrent_A = I_battery;
        % row.EstimatedEndurance_min = endurance_min;
        % row.EstimatedDCInputPerRotor_A = I_rotor;
        % row.PassesBasicElectricalScreen = RPM_hover < RPM_no_load && ...
        % I_rotor <= ESC_current_limit && I_battery <= I_battery_max;
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
HoverMatched = struct2table(matched_rows);

writetable(HoverAll,'APC_hover_all_points_matlab.csv');
writetable(HoverMatched,'APC_hover_matched_matlab.csv');

disp("kelar")

% fprintf('NOTE: APC-derived coefficients and assumed masses, efficiency, limits.\n');
% fprintf('Not validated Gemfan/SunnySky performance predictions.\n');
function T = apcHoverThrust(rpm, prop_name, rho, D)
data = apcProp(prop_name,rpm,0);
T = data.Ct*rho*(rpm/60)^2*D^4;
end