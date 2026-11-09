%% APC propeller data - examples
clear; clc;

% Available propellers
apcProp('list')

% Original data for standard 4000--9000 RPM operating window
T = apcProp('data','9x4.5MR');
disp(T(1:min(5,height(T)),:))

% Full source data across ALL RPMs
T_all = apcProp('data','9x4.5MR','all');

% Interpolation: RPM = 6500, axial advance ratio J = 0.12
p = apcProp('9x4.5MR',6500,0.12);
fprintf('CT = %.5f, CP = %.5f, Pe = %.5f\n',p.Ct,p.Cp,p.Pe);
fprintf('V = %.3f m/s (%.3f mph)\n',p.V_mps,p.V_mph);
fprintf('Power = %.3f W, Torque = %.5f Nm, Thrust = %.4f N\n', ...
        p.Power_W,p.Torque_Nm,p.Thrust_N);
fprintf('THR/PWR = %.3f g/W\n',p.ThrustPerPower_gW);

% Or specify axial velocity directly instead of J
p2 = apcProp('velocity','10x4.5MR',6500,5,'mps');
fprintf('J = %.5f, CT = %.5f\n',p2.J,p2.Ct);

% Hover: use J=0
hover = apcProp('9x4.7SF',6000,0);
fprintf('Hover coefficient CT0=%.5f, CP0=%.5f\n',hover.Ct,hover.Cp);

% IMPORTANT: To solve forward RPM with oblique-flow correction,
% use J_axial as the query and then apply your separate correction.
