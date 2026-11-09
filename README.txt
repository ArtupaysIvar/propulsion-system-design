APC PER3 MATLAB LOOKUP PACKAGE
==============================
FILES:
- apcProp.m: MATLAB loader and global-call interpolation function.
- demo_apc.m: usage examples.
- *_APC.csv: faithfully parsed source numeric values for all RPM blocks.
- PER3*.dat: original supplied raw files unchanged.

SELECTED ANALYSIS RPM RANGE: 4000 to 9000 inclusive (editable in apcProp.m).
NO RPM BLOCKS ARE REMOVED from the CSVs or .dat files.

USAGE:
  apcProp('list')
  T = apcProp('data','9x4.5MR');
  T_all = apcProp('data','9x4.5MR','all');
  q = apcProp('9x4.5MR',6500,0.12);
  q = apcProp('velocity','9x4.5MR',6500,5,'mps');

q fields:
  Propeller, RPM, J, V_mph, V_mps, Pe, Ct, Cp,
  Power_hp, Torque_inlbf, Thrust_lbf,
  Power_W, Torque_Nm, Thrust_N, ThrustPerPower_gW,
  Mach, Reyn, FOM.

SI note: original APC files ALREADY include power in W, torque in N-m and
thrust in N. These source values are preserved unchanged, no recalculation.
V_mps is derived from the original MPH field via factor 0.44704.
ThrustPerPower_gW is source data in grams per watt (not an SI quantity).

Rows in original source with only two numbers (speed, advance ratio):
  remaining columns are blank in CSV and become NaN in MATLAB; none are
  fabricated or extrapolated. No interpolation outside available J ranges.

APC source files are calculated/estimated propeller data, NOT electrically
measured motor+ESC power. The V field refers to axial flow along the propeller
axis. For tilted multirotor flight, querying at J_axial supplies only the
axial baseline; it does not include oblique inflow corrections.

The 4000-9000 RPM window is a convenient comparison band, NOT a verified
motor/ESC/propeller operational limit. Validate it against actual hardware.
