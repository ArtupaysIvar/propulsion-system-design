function result = apcProp(varargin)
% APCPROP Load and interpolate original APC PER3 propeller performance data.
%
% DEFAULT OPERATING WINDOW: 4000--9000 RPM (user configurable below).
% The complete original data for every RPM remain in *_APC.csv and .dat.
%
% apcProp('list')                           % available propellers
% apcProp('data','9x4.5MR')                % raw table, default RPM window
% apcProp('data','9x4.5MR','all')          % ALL original RPM rows
% apcProp('9x4.5MR',6500,0.10)            % interpolate using axial J
% apcProp('velocity','9x4.5MR',6500,5)   % velocity in m/s, axial flow
% apcProp('velocity','9x4.5MR',6500,12,'mph') % velocity in MPH
%
% Coefficients: Ct/Cp/Pe and other data are from APC PER3, not Pollet.
% Power_W = shaft power, not input electrical power. Axial flow only.
% The parser retains the source's rounded numbers without recalculating them.
%
% CAUTION: APC files provide calculations, not experimental measurements.
% No extrapolation in J or RPM. Source records with missing data are excluded
% from interpolation for that field but remain in the source table.

persistent cache
if isempty(cache)
    cache = struct();
end
baseDir = fileparts(mfilename('fullpath'));
validNames = {'9x4.5MR','9x4.7SF','10x4.5MR','10x4.7SF'};

if nargin == 1 && strcmpi(varargin{1},'list')
    result = validNames(:);
    return;
end

if nargin < 2
    error('Usage: apcProp(prop,RPM,J) or apcProp(''data'',prop).');
end

isData = ischar(varargin{1}) && strcmpi(varargin{1},'data');
isVelocity = ischar(varargin{1}) && strcmpi(varargin{1},'velocity');
if isData || isVelocity
    prop = char(varargin{2});
else
    prop = char(varargin{1});
end
ix = find(strcmpi(validNames,prop),1);
if isempty(ix)
    error('Unknown propeller "%s". Use apcProp(''list'').',prop);
end
prop = validNames{ix};
key = ['prop' num2str(ix)];
if ~isfield(cache,key)
    file = fullfile(baseDir,[prop '_APC.csv']);
    if ~isfile(file)
        error('Missing CSV: %s. Keep apcProp.m and the CSVs in one folder.',file);
    end
    cache.(key) = readtable(file,'VariableNamingRule','preserve');
end
T = cache.(key);

% Change these values if you want a different NORMAL OPERATING RANGE.
RPM_MIN = 4000;
RPM_MAX = 9000;

if isData
    if nargin >= 3 && ischar(varargin{3}) && strcmpi(varargin{3},'all')
        result = T;
    else
        result = T(T.RPM>=RPM_MIN & T.RPM<=RPM_MAX,:);
    end
    return;
end

if isVelocity
    if nargin < 4
        error('Usage: apcProp(''velocity'',prop,RPM,velocity,[unit]).');
    end
    rpm = varargin{3};
    speed = varargin{4};
    unit = 'mps';
    if nargin>=5
        unit = lower(char(varargin{5}));
    end
    if strcmp(unit,'mph')
        speed = speed * 0.44704;
    elseif ~strcmp(unit,'mps')
        error('Velocity units must be ''mps'' or ''mph''.');
    end
    propDiam_in = [9,9,10,10];
    D = propDiam_in(ix)*0.0254;
    J = speed/((rpm/60)*D); % velocity ALONG propeller axis
else
    if nargin<3
        error('Usage: apcProp(prop,RPM,J).');
    end
    rpm=varargin{2};
    J=varargin{3};
end
if ~isscalar(rpm) || ~isscalar(J) || ~isfinite(rpm) || ~isfinite(J)
    error('RPM and J must be finite scalars.');
end
if rpm < RPM_MIN || rpm > RPM_MAX
    error('RPM=%g is outside selected operating window [%g,%g].',rpm,RPM_MIN,RPM_MAX);
end

rValues = unique(T.RPM(T.RPM>=RPM_MIN & T.RPM<=RPM_MAX));
lo = find(rValues<=rpm,1,'last');
hi = find(rValues>=rpm,1,'first');
if isempty(lo) || isempty(hi)
    error('RPM outside the available data range.');
end
rpm1 = rValues(lo);
rpm2 = rValues(hi);

% Interpolate J within each RPM slice, then linearly between RPM slices.
% Each property is interpolated independently to respect missing source data.
fields = {'V_mph','Pe','Ct','Cp','Power_hp','Torque_inlbf', ...
          'Thrust_lbf','Power_W','Torque_Nm','Thrust_N', ...
          'ThrustPerPower_gW','Mach','Reyn','FOM'};
result = struct('Propeller',prop,'RPM',rpm,'J',J);
for i=1:numel(fields)
    name=fields{i};
    q1=atJ(T,rpm1,J,name);
    if rpm1==rpm2
        val=q1;
    else
        q2=atJ(T,rpm2,J,name);
        if ~isfinite(q1) || ~isfinite(q2)
            val=NaN;
        else
            val=q1+(q2-q1)*(rpm-rpm1)/(rpm2-rpm1);
        end
    end
    result.(name)=val;
end
result.V_mps = result.V_mph * 0.44704;
% V_mph and J are separately rounded in APC source; consequently the
% returned V_mps may differ slightly from J*(RPM/60)*D.
if ~isfinite(result.Ct) || ~isfinite(result.Cp)
    warning('No complete CT/CP data at J=%g for the selected RPM(s). Returned NaN.',J);
end
end

function val=atJ(T,rpm,J,name)
rows=T(T.RPM==rpm,:);
x=rows.J;
y=rows.(name);
valid=isfinite(x)&isfinite(y);
x=x(valid); y=y(valid);
[x,ix]=unique(x,'stable'); y=y(ix);
[x,ix]=sort(x); y=y(ix);
if isempty(x) || J<min(x) || J>max(x)
    val=NaN; return;
end
if numel(x)==1
    if J==x
        val=y;
    else
        val=NaN;
    end
elseif numel(x)==2
    val=interp1(x,y,J,'linear');
else
    val=interp1(x,y,J,'pchip');
end
end
