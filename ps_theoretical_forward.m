
%% MULTIROTOR HOVER EVALUATION (MATLAB equivalent of calculate_hover.py)
% Requires MATLAB with table support (no additional toolboxes).
% APC-based static coefficients; battery and electrical inputs are illustrative.
clear; clc; close all;

g = 9.80665;
rho = 1.225;
N_rotors = 4;
m_without_battery = 1.20;  % kg, ASSUMED
m_payload = 0.0;           % kg
avionics_power = 20;       % W, ASSUMED
usable_fraction = 0.750;    % ASSUMED
eta_ESC_motor = 0.8;      % combined efficiency, ASSUMED
motor_KV = 880;            % RPM/V
ESC_current_limit = 30;    % A, assumed battery-side per-rotor limit

r_prime = 0.75;
sigma_e = 0.0707; % not in pollet

% T_hover_per_rotor = W/N_rotors;
% T_design_per_rotor = TWR_required*W/N_rotors;

% variabel
V_inf = 1e-6:.5:15.1;       
% alpha = deg2rad(1:4:45); % Incidence angle (rad)
propellers = [9 4.5; 9 4.7; 10 4.5; 10 4.7];
RPM_values = 4000:500:9000;
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
        W = m_total * g;
        T_hover = m_total * g / N_rotors;
        V_battery = 3.7 * S;   % nominal voltage
        E_usable = V_battery * Ah * usable_fraction; % Wh
        I_battery_max = Ah * C_rating;
        RPM_no_load = motor_KV * V_battery; % ideal ceiling, NOT motor validation

        theta_FP = 0;
        Cd_frame = 0.425;
        Cl_frame = 0.193;
        S_top = 0.226;
        S_front = 0.09;

        % Same columns and calculation as Python 'common' fields
        common = struct('Propeller',sprintf('%gx%g',diameter_in,pitch_in), ...
            'Diameter_in',diameter_in,'Pitch_in',pitch_in, ...
            'Battery',battery_name,'Battery_S',S,'Battery_Ah',Ah, ...
            'BatteryMass_kg',battery_mass,'TotalMass_kg',m_total, ...
            'NominalVoltage_V',V_battery, ...
            'RequiredHoverThrustPerRotor_N',T_hover, ...
            'UsableBatteryEnergy_Wh',E_usable);
             % 'CT_static',CT0,'CP_static',CP0, ...
         

        % All sampled operating points
        for k = 1:numel(RPM_values)
            RPM = RPM_values(k);
            n = RPM / 60;

            for x = 1:numel(V_inf)
                % ini buat cari alpha dan thrust pake equilibrium

                S_ref_fun = @(a) S_top*sin(a) + S_front*cos(a);
                K = 0.5*rho*V_inf(x)^2;
                
                Df_fun = @(a) K*Cd_frame*S_ref_fun(a);
                Lf_fun = @(a) K*Cl_frame*S_ref_fun(a);

                A_fun = @(a) W + Df_fun(a)*sin(theta_FP) ...
                            + Lf_fun(a)*cos(theta_FP);

                B_fun = @(a) Df_fun(a)*cos(theta_FP) ...
                            - Lf_fun(a)*sin(theta_FP);

                angle_equation = @(a) a-theta_FP-atan2(B_fun(a),A_fun(a));

                alpha = fzero(angle_equation,[0 pi/2]);

                T_required = hypot(A_fun(alpha), ...
                                B_fun(alpha))/N_rotors;
                                
                % ini buat cari CT sama CP
                J = V_inf(x) / (n * D);
                J_axial = J * sin(alpha);

                if J_axial >= 0.95*min(J0T,J0P)
                    continue;
                end

                CT_axial = 0.02791 - 0.06543*J_axial + 0.11867*beta ...
                    + 0.27334*beta^2 - 0.28852*beta^3 ...
                    + 0.02104*J_axial^3 - 0.23504*J_axial^2 + 0.18677*beta^2*J_axial;
                CP_axial = 0.01813 - 0.06218*beta + 0.00343*J_axial ...
                    + 0.35712*beta^2 - 0.23774*beta^3 ...
                    + 0.07549*beta*J_axial - 0.1235*J_axial^2;

                % Incidence correction
                delta = (3/2)*cos(beta_prime) * ...
                    (1 + (sigma_e/tan(beta_prime)) * ...
                    (1 + sqrt(1 + 2*tan(beta_prime)/sigma_e)) * ...
                    (1 - sin(alpha)));
                numerator = (J*cos(alpha)/(pi*r_prime))^2;

                eta_T = 1 + numerator*delta / ...
                    (2*(1 - J_axial/J0T));

                eta_P = 1 + numerator*delta / ...
                    (2*(1 - J_axial/J0P));

                CT_corrected = eta_T*CT_axial;
                CP_corrected = eta_P*CP_axial;

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
                row.Alpha_deg = rad2deg(alpha);
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
                
                row.EnoughThrust = Thrust >= T_required;
                row.RequiredThrustPerRotor_N = T_required;
                row.ThrustError_N = Thrust - T_required;
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


        %% MATCHED RPM FOR FORWARD FLIGHT
        for x = 1:numel(V_inf)

            V = V_inf(x);
            theta_FP = 0;

            K = 0.5*rho*V^2;
            S_ref_fun = @(a) S_top*sin(a) + S_front*cos(a);

            Df_fun = @(a) K*Cd_frame*S_ref_fun(a);
            Lf_fun = @(a) K*Cl_frame*S_ref_fun(a);

            A_fun = @(a) W + Df_fun(a)*sin(theta_FP) ...
                        + Lf_fun(a)*cos(theta_FP);

            B_fun = @(a) Df_fun(a)*cos(theta_FP) ...
                        - Lf_fun(a)*sin(theta_FP);

            angle_equation = @(a) a-theta_FP ...
                - atan2(B_fun(a),A_fun(a));

            alpha = fzero(angle_equation,[0 pi/2]);

            T_required = hypot(A_fun(alpha), ...
                               B_fun(alpha))/N_rotors;

            % Initial estimate using static CT
            RPM_forward = 60*sqrt(T_required/(CT0*rho*D^4));

            %% Fixed-point iteration
            for iter = 1:100

                n = RPM_forward/60;
                J = V/(n*D);
                J_axial = J*sin(alpha);

                if J_axial >= 0.95*min(J0T,J0P)
                    break;
                end

                CT_axial = 0.02791 - 0.06543*J_axial ...
                    + 0.11867*beta + 0.27334*beta^2 ...
                    - 0.28852*beta^3 + 0.02104*J_axial^3 ...
                    - 0.23504*J_axial^2 ...
                    + 0.18677*beta^2*J_axial;

                delta = (3/2)*cos(beta_prime) * ...
                    (1 + sigma_e/tan(beta_prime) * ...
                    (1 + sqrt(1+2*tan(beta_prime)/sigma_e)) ...
                    *(1-sin(alpha)));

                numerator = (J*cos(alpha)/(pi*r_prime))^2;

                eta_T = 1 + numerator*delta / ...
                    (2*(1-J_axial/J0T));

                CT_corrected = eta_T*CT_axial;

                if ~isfinite(CT_corrected) || CT_corrected <= 0
                    break;
                end

                RPM_new = 60*sqrt(T_required / ...
                    (CT_corrected*rho*D^4));

                error_RPM = abs(RPM_new-RPM_forward);
                RPM_forward = RPM_new;

                if error_RPM < 0.01
                    break;
                end
            end

            % Validate convergence before using result
            if iter == 100 || ~isfinite(RPM_forward)
                continue;
            end

            %% Recalculate at final RPM
            n = RPM_forward/60;
            J = V/(n*D);
            J_axial = J*sin(alpha);

            if J_axial >= 0.95*min(J0T,J0P)
                continue;
            end

            CT_axial = 0.02791 - 0.06543*J_axial ...
                + 0.11867*beta + 0.27334*beta^2 ...
                - 0.28852*beta^3 + 0.02104*J_axial^3 ...
                - 0.23504*J_axial^2 ...
                + 0.18677*beta^2*J_axial;

            CP_axial = 0.01813 - 0.06218*beta ...
                + 0.00343*J_axial + 0.35712*beta^2 ...
                - 0.23774*beta^3 + 0.07549*beta*J_axial ...
                - 0.1235*J_axial^2;

            numerator = (J*cos(alpha)/(pi*r_prime))^2;

            eta_T = 1 + numerator*delta / ...
                (2*(1-J_axial/J0T));

            eta_P = 1 + numerator*delta / ...
                (2*(1-J_axial/J0P));

            CT_corrected = eta_T*CT_axial;
            CP_corrected = eta_P*CP_axial;

            Thrust = CT_corrected*rho*n^2*D^4;
            Pshaft = CP_corrected*rho*n^3*D^5;

            if ~isfinite(Thrust) || ~isfinite(Pshaft) ...
                    || Thrust <= 0 || Pshaft <= 0 ...
                    || abs(Thrust-T_required) > 0.001*T_required
                continue;
            end

            Torque = Pshaft/(2*pi*n);
            Pelec_rotor = Pshaft/eta_ESC_motor;
            Pelec_total = N_rotors*Pelec_rotor + avionics_power;
            I_rotor = Pelec_rotor/V_battery;
            I_battery = Pelec_total/V_battery;
            endurance_min = E_usable/Pelec_total*60;

            %% Save matched results
            row = common;
            row.RPM = RPM_forward;
            row.V_inf = V;
            row.Alpha_deg = rad2deg(alpha);
            row.RequiredThrustPerRotor_N = T_required;
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

            row.PassesBasicElectricalScreen = ...
                RPM_forward < RPM_no_load && ...
                I_rotor <= ESC_current_limit && ...
                I_battery <= I_battery_max;
            row.ThrustError_N = Thrust - T_required;
            row.BelowIdealNoLoadRPM = RPM_forward < RPM_no_load;
            row.WithinAssumedESCDCInputLimit = I_rotor <= ESC_current_limit;
            row.WithinRatedBatteryC = I_battery <= I_battery_max;

            if isempty(matched_rows)
                matched_rows = row;
            else
                matched_rows(end+1) = row;
            end
        end
    end
end

%% 4. Save and display
ForwardAll = struct2table(all_rows);
ForwardMatched = struct2table(matched_rows);

size(ForwardAll)
size(ForwardMatched)

writetable(ForwardAll,'theory_forward_all_points_matlab.csv');
writetable(ForwardMatched,'theory_forward_matched_matlab.csv');

fprintf('kelar\n');
% fprintf('NOTE: APC-derived coefficients and assumed masses, efficiency, limits.\n');
% fprintf('Not validated Gemfan/SunnySky performance predictions.\n');

