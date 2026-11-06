%% GEMFAN 9047 + SUNNY SKY X2216 880KV: HOVER AND FORWARD FLIGHT
% References:
% Pollet et al., "Design Optimization of Multirotor Drones in Forward Flight",
% ICAS 2020, Eqs. (3)-(11).
% Nam et al., "Electric Propulsion System Analysis and Optimization for
% Multi-Rotor Drones", IEEE Aerospace Conference 2024, Eq. (1).
%
% IMPORTANT LIMITATIONS
% (1) Pollet's CT/CP fits use APC data, NOT Gemfan 9047 measurements.
% (2) Forward flight uses the AXIAL part of Pollet's model. The full
%     incidence factors of Eq. (9) require a blade-dependent solidity
%     correction delta(alpha), which is unavailable for Gemfan 9047.
% (3) Electrical efficiency is assumed, NOT measured. Calibrate with
%     real motor+ESC+propeller test data before publishing predictions.
% (4) This is level, steady forward flight, not transient maneuvering.
% (5) The manufacturer's maximum thrust/current figures are NOT inferred.
clear; clc; close all;

%% 1. AIRCRAFT AND PROPULSION INPUTS
N_rotors = 4;
g = 9.80665; rho = 1.225;
m_empty = 1.70;           % kg, includes battery; user-editable
m_payload = 0.0;          % kg
m_total = m_empty + m_payload;
W = m_total*g;
TWR_required = 2.0;

motor_name = 'SunnySky X2216 880KV';
motor_KV = 880;           % RPM/V
motor_max_current = 20;   % A, FROM ORIGINAL SCRIPT: VERIFY YOUR VARIANT
ESC_max_current = 30;     % A, FROM ORIGINAL SCRIPT: VERIFY

prop_name = 'Gemfan 9047 CF Nylon';
prop_diameter_in = 9.0;
prop_pitch_in = 4.7;       % 9047 = 9 x 4.7 inches, not 9 x 4.5
D = prop_diameter_in*0.0254;
pitch = prop_pitch_in*0.0254;
beta = pitch/D;           % P/D ratio, not physical blade pitch angle
A_prop = pi*(D/2)^2;
A_total = N_rotors*A_prop;

battery_cells = 4;
battery_capacity_Ah = 6.5;
battery_voltage = 14.8;
battery_max_C = 60;
usable_fraction = 0.80; % assumption, not Nam's universal result
avionics_power = 10;     % W
eta_ESC_motor_assumed = 0.75; % ILLUSTRATIVE; NOT measured for this setup
E_usable_Wh = battery_voltage*battery_capacity_Ah*usable_fraction;

%% 2. FORWARD FLIGHT DRAG MODEL (Pollet Eqs. 10-11)
Cd_frame = 1.0;           % assumed; replace with measured/CFD result
Cl_frame = 0.0;           % assumed no airframe lift/downforce
S_top = 0.15;             % m^2, ASSUMED projected area, replace
S_front = 0.06;           % m^2, ASSUMED projected area, replace

speed_mps = 0:0.5:15;     % steady level forward-speed sweep
rpm_grid = (1000:10:16000)'; % rotor RPM grid; not a verified motor limit

%% 3. POLLET ET AL. EQ. (8): APC-DERIVED CT AND CP POLYNOMIALS
% J_axial = V*sin(alpha)/(n*D); n = RPM/60 in rev/s.
% alpha = angle between rotor disk and freestream (Pollet Fig. 1).
% Here alpha is determined from weight/drag equilibrium.
CT_axial = @(J) 0.02791 - 0.06543.*J + 0.11867.*beta ...
    + 0.27334.*beta.^2 - 0.28852.*beta.^3 ...
    + 0.02104.*J.^3 - 0.23504.*J.^2 + 0.18677.*beta.^2.*J;
CP_axial = @(J) 0.01813 - 0.06218.*beta + 0.00343.*J ...
    + 0.35712.*beta.^2 - 0.23774.*beta.^3 ...
    + 0.07549.*beta.*J - 0.1235.*J.^2;

% Pollet Eqs. (6a-6b): T=CT*rho*n^2*D^4, Pshaft=CP*rho*n^3*D^5.
% Q=Pshaft/(2*pi*n), and Nam Eq. (1): Pbattery=Pshaft/eta_ESC_motor.
% Full incidence correction (Pollet Eq. 9), NOT activated here:
% CT(alpha,J)=eta_T(alpha,J)*CT_axial(J_axial)
% CP(alpha,J)=eta_P(alpha,J)*CP_axial(J_axial)
% eta_T,P=1+(J*cos(alpha)/(pi*r_prime))^2 / ...
%           (2*(1-J_axial/J0_T,P)*delta(alpha))
% r_prime=0.75, J0_T,P are zero-thrust/power axial advance ratios.
% delta(alpha) is blade-dependent and cannot be specified from a 9047
% label alone. Assuming it without validation could be misleading.

%% 4. STATIC PROPELLER PERFORMANCE (Pollet axial surrogate, J=0)
rpm = rpm_grid;
n = rpm/60;
CT0 = CT_axial(0);
CP0 = CP_axial(0);
T_static = CT0*rho.*n.^2*D^4;    % N/rotor
Pshaft_static = CP0*rho.*n.^3*D^5; % W/rotor
Q_static = Pshaft_static./(2*pi*n); % Nm/rotor
Pelec_static = Pshaft_static/eta_ESC_motor_assumed;
I_static = Pelec_static/battery_voltage;
T_hover_per_rotor = W/N_rotors;
T_design_per_rotor = TWR_required*W/N_rotors;

if CT0<=0 || CP0<=0
    error('Non-physical static coefficients. Model not applicable to this beta.');
end

rpm_hover = sqrt(T_hover_per_rotor/(CT0*rho*D^4))*60;
Pshaft_hover = interp1(rpm,Pshaft_static,rpm_hover,'linear',NaN);
Pelec_hover = Pshaft_hover/eta_ESC_motor_assumed;
Ptotal_hover = N_rotors*Pelec_hover+avionics_power;
endurance_hover = E_usable_Wh/Ptotal_hover*60;

fprintf('\nMODEL: %s / %s\n',motor_name,prop_name);
fprintf('Mass %.3f kg; pitch/diameter beta %.4f\n',m_total,beta);
fprintf('APC-surrogate static CT=%.5f, CP=%.5f (NOT measured Gemfan values)\n',CT0,CP0);
fprintf('Hover required: %.3f N/rotor; RPM %.0f\n',T_hover_per_rotor,rpm_hover);
fprintf('Hover shaft power %.1f W/rotor; assumed electrical %.1f W/rotor\n',Pshaft_hover,Pelec_hover);
fprintf('Estimated hover endurance %.1f min (assumption-dependent)\n',endurance_hover);
fprintf('Design target (TWR %.1f): %.2f N/rotor\n',TWR_required,T_design_per_rotor);

%% 5. LEVEL FORWARD-FLIGHT FORCE EQUILIBRIUM (Pollet Eqs. 3-5)
% Df=0.5*rho*V^2*Cd*Sref, Lf=0.5*rho*V^2*Cl*Sref.
% Sref=S_top*sin(alpha)+S_front*cos(alpha).
% alpha=atan2(Df,W+Lf) for level flight (flight path angle=0).
% T_required_per_rotor=sqrt((W+Lf)^2+Df^2)/N_rotors.
% Iterate alpha because Sref itself depends on alpha.
NV = numel(speed_mps);
alpha_rad = NaN(1,NV); drag_N = NaN(1,NV);
lift_N = NaN(1,NV); Treq_N = NaN(1,NV);
RPM_fwd = NaN(1,NV); Jax_fwd = NaN(1,NV);
CT_fwd = NaN(1,NV); CP_fwd = NaN(1,NV);
Pshaft_fwd = NaN(1,NV); Pelec_fwd = NaN(1,NV);
Ptotal_fwd = NaN(1,NV); endurance_fwd = NaN(1,NV);
range_km = NaN(1,NV);

for k = 1:NV
    V = speed_mps(k);
    alpha = 0;
    for it = 1:100
        Sref = S_top*sin(alpha)+S_front*cos(alpha);
        Df = 0.5*rho*V^2*Cd_frame*Sref;
        Lf = 0.5*rho*V^2*Cl_frame*Sref; % positive downward
        anew = atan2(Df,W+Lf);
        if abs(anew-alpha)<1e-10; alpha=anew; break; end
        alpha = anew;
    end
    alpha_rad(k)=alpha;
    drag_N(k)=Df; lift_N(k)=Lf;
    Treq_N(k)=hypot(W+Lf,Df)/N_rotors;

    % The propeller's inflow is generally OBLIQUE in forward flight.
    % Use Pollet's axial polynomial only: J_axial=V*sin(alpha)/(nD).
    % This omits incidence correction eta_T/P and thus is not full Eq. (9).
    ngrid = rpm_grid/60;
    Jax_grid = V*sin(alpha)./(ngrid*D);
    CTgrid = CT_axial(Jax_grid);
    CPgrid = CP_axial(Jax_grid);
    Tgrid = CTgrid.*rho.*ngrid.^2*D^4;
    valid = isfinite(Tgrid)&isfinite(CPgrid)&CTgrid>0&CPgrid>0&Jax_grid>=0&Jax_grid<=0.5;
    e = Tgrid-Treq_N(k);
    idx = find(valid(1:end-1)&valid(2:end)&e(1:end-1)<=0&e(2:end)>=0,1);
    if isempty(idx); continue; end
    % interpolate bracketing RPM; recalculate coefficients at interpolated RPM
    RPM_fwd(k)=interp1(Tgrid(idx:idx+1),rpm_grid(idx:idx+1),Treq_N(k));
    nr=RPM_fwd(k)/60;
    Jax_fwd(k)=V*sin(alpha)/(nr*D);
    CT_fwd(k)=CT_axial(Jax_fwd(k));
    CP_fwd(k)=CP_axial(Jax_fwd(k));
    Pshaft_fwd(k)=CP_fwd(k)*rho*nr^3*D^5;
    Pelec_fwd(k)=Pshaft_fwd(k)/eta_ESC_motor_assumed;
    Ptotal_fwd(k)=N_rotors*Pelec_fwd(k)+avionics_power;
    endurance_fwd(k)=60*E_usable_Wh/Ptotal_fwd(k);
    range_km(k)=V*endurance_fwd(k)*60/1000;
end

fprintf('\nFORWARD FLIGHT (APC SURROGATE; NO INCIDENCE CORRECTION)\n');
forward_table=table(speed_mps(:),rad2deg(alpha_rad(:)),drag_N(:),Treq_N(:), ...
    RPM_fwd(:),Jax_fwd(:),CT_fwd(:),CP_fwd(:),Ptotal_fwd(:), ...
    endurance_fwd(:),range_km(:), ...
    'VariableNames',{'Speed_mps','DiskAngle_deg','Drag_N','ThrustPerRotor_N', ...
    'RPM','Jaxial','CT','CP','TotalElectricPower_W','Endurance_min','Range_km'});
disp(forward_table);

%% 6. OPTIONAL USER-SUPPLIED BENCH DATA
% Supply a CSV with columns: Thrust_N, RPM, Voltage_V, Current_A,
% and optionally Torque_Nm, measured for the EXACT motor/prop/ESC.
% Leave blank to avoid presenting fictional 'manufacturer' data.
bench_csv = '';
if ~isempty(bench_csv)
    test = readtable(bench_csv);
    required = {'Thrust_N','RPM','Voltage_V','Current_A'};
    assert(all(ismember(required,test.Properties.VariableNames)), ...
        'Bench CSV must contain Thrust_N, RPM, Voltage_V, Current_A');
    bench_Pelec = test.Voltage_V.*test.Current_A;
    bench_CT = test.Thrust_N./(rho.*(test.RPM/60).^2*D^4);
    fprintf('\nMeasured bench CT values:\n'); disp(bench_CT);
    if ismember('Torque_Nm',test.Properties.VariableNames)
        bench_Pshaft = 2*pi*(test.RPM/60).*test.Torque_Nm;
        bench_CP = bench_Pshaft./(rho.*(test.RPM/60).^3*D^5);
        bench_eta = bench_Pshaft./bench_Pelec;
        disp(table(bench_CP,bench_eta));
    else
        fprintf('No measured torque: cannot compute true CP or ESC-motor efficiency.\n');
    end
end

%% 7. PAYLOAD VS HOVER ENDURANCE (THEORETICAL, NOT TESTED)
payload_range = 0:0.1:2;
TWR_payload = NaN(size(payload_range));
endurance_payload = NaN(size(payload_range));
% TWR requires verified maximum thrust; do not use arbitrary RPM as max.
verified_max_thrust_N_per_rotor = NaN; % insert actual bench value
for k=1:numel(payload_range)
    Wk=(m_empty+payload_range(k))*g;
    Treq=Wk/N_rotors;
    nh=sqrt(Treq/(CT0*rho*D^4));
    Pk=CP0*rho*nh^3*D^5;
    if nh*60<=max(rpm_grid)
        endurance_payload(k)=60*E_usable_Wh/(N_rotors*Pk/eta_ESC_motor_assumed+avionics_power);
    end
    if ~isnan(verified_max_thrust_N_per_rotor)
        TWR_payload(k)=N_rotors*verified_max_thrust_N_per_rotor/Wk;
    end
end

%% 8. PLOTS
figure('Name','Static propulsion surrogate');
tiledlayout(2,2);
nexttile; plot(rpm,T_static,'LineWidth',1.4); grid on;
xlabel('RPM'); ylabel('Thrust (N/rotor)'); title('Static thrust: APC-derived fit');
nexttile; plot(rpm,Pshaft_static,'LineWidth',1.4); grid on;
xlabel('RPM'); ylabel('Shaft power (W/rotor)'); title('Static shaft power');
nexttile; plot(T_static,Pelec_static,'LineWidth',1.4); grid on;
xlabel('Thrust (N/rotor)'); ylabel('Assumed electrical power (W/rotor)');
title('Thrust vs assumed input power');
nexttile; plot(rpm,Q_static,'LineWidth',1.4); grid on;
xlabel('RPM'); ylabel('Torque (N m/rotor)'); title('Static torque');

figure('Name','Forward flight: axial-only surrogate');
tiledlayout(2,3);
nexttile; plot(speed_mps,rad2deg(alpha_rad),'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Disk angle (deg)'); title('Equilibrium tilt');
nexttile; plot(speed_mps,RPM_fwd,'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Rotor RPM'); title('Required RPM');
nexttile; plot(speed_mps,CT_fwd,speed_mps,CP_fwd,'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Coefficient'); legend('C_T','C_P');
title('Pollet axial coefficients');
nexttile; plot(speed_mps,Ptotal_fwd,'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Electric power (W)'); title('Assumed electric power');
nexttile; plot(speed_mps,endurance_fwd,'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Endurance (min)'); title('Assumed endurance');
nexttile; plot(speed_mps,range_km,'LineWidth',1.4); grid on;
xlabel('Speed (m/s)'); ylabel('Range (km)'); title('Ideal still-air range');
sgtitle('Preliminary forward model: incidence correction NOT included');

figure('Name','Payload study');
plot(payload_range,endurance_payload,'LineWidth',1.5); grid on;
xlabel('Payload (kg)'); ylabel('Hover endurance (min)');
title('Payload versus estimated hover endurance');

fprintf('\nIMPORTANT: No full non-axial incidence correction (delta(alpha) unknown).\n');
fprintf('Replace assumed Cd, S_top, S_front, efficiency with measured values.\n');
fprintf('Do not report modelled TWR without verified maximum thrust.\n');
