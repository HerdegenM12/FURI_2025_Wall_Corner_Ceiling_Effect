function results = rise_flight_analysis(csvFile)
% RISE_FLIGHT_ANALYSIS
% ASU RISE Lab FURI - UAV flight / rotor stability analysis
% Matthew Herdegen
%
% Usage:
%   results = rise_flight_analysis("flight_data.csv");
%
% The CSV should contain a time column and one or more motor RPM columns.
% Position, attitude, rate, and commanded-position columns are optional.
%
% Core force model:
%   T_i = kT * omega_i^2
%   omega_i = RPM_i * 2*pi/60
%
% IMPORTANT:
% kT must come from the motor/propeller model or a thrust-stand/simulation
% calibration. Do not treat the default placeholder as a measured value.

%% ------------------------------------------------------------------------
%  USER CONFIGURATION
% -------------------------------------------------------------------------

cfg.mass_kg = 2.00;                  % Vehicle mass [kg] - CHANGE THIS
cfg.g = 9.80665;                    % Gravity [m/s^2]

% Motor/propeller thrust coefficient.
% Units: N/(rad/s)^2 when using T = kT*omega^2.
% CHANGE THIS using simulation model data or thrust calibration.
cfg.kT_N_per_rad_s2 = 1.20e-5;

% Optional rotor drag/yaw-torque coefficient:
% Q_i = kQ * omega_i^2
% Set NaN if unknown.
cfg.kQ_Nm_per_rad_s2 = NaN;

% Quadrotor geometry used for estimated body moments.
% Assumed X configuration and motor order:
%   1 = Front Left
%   2 = Front Right
%   3 = Rear Right
%   4 = Rear Left
cfg.armLength_m = 0.23;

% Rotor direction multiplier for yaw reaction torque.
% Adjust this to match your vehicle.
% Example: [CCW CW CCW CW] represented as [+1 -1 +1 -1].
cfg.rotorYawDirection = [1 -1 1 -1];

% Coordinate convention used here:
% body x = forward, body y = right, body z = up.
% MAVROS local position is commonly ENU, so local z is normally positive up.
cfg.localZPositiveUp = true;

% If attitude columns are supplied as roll/pitch/yaw:
cfg.attitudeInputDegrees = true;

% Phase-detection thresholds
cfg.groundAltitude_m = 0.25;
cfg.climbRate_mps = 0.15;
cfg.hoverVerticalRate_mps = 0.15;
cfg.hoverHorizontalSpeed_mps = 0.25;

% Optional manually selected analysis windows [start_s end_s].
% Add or replace windows to study ground effect, wall effect, gusts, etc.
cfg.windows_s = [
    0  10
    10 20
    20 30
];
cfg.windowNames = ["Window_1","Window_2","Window_3"];

% Output directory
cfg.outputDir = "rise_analysis_results";

%% ------------------------------------------------------------------------
%  LOAD DATA
% -------------------------------------------------------------------------

if nargin < 1 || strlength(string(csvFile)) == 0
    csvFile = "flight_data.csv";
end

if ~isfile(csvFile)
    error("CSV file not found: %s", csvFile);
end

T = readtable(csvFile, "VariableNamingRule", "preserve");
vars = string(T.Properties.VariableNames);

if ~isfolder(cfg.outputDir)
    mkdir(cfg.outputDir);
end

%% ------------------------------------------------------------------------
%  TIME
% -------------------------------------------------------------------------

timeName = findColumn(vars, ...
    ["time","time_s","timestamp","timestamp_s","t","elapsed_time","elapsed_s"]);

if timeName == ""
    error(["No time column found. Expected something like " ...
           "'time', 'time_s', 'timestamp', or 't'."]);
end

tRaw = T.(timeName);
t = normalizeTime(tRaw);

if numel(t) < 3
    error("At least three time samples are required.");
end

dt = median(diff(t), "omitnan");
if ~isfinite(dt) || dt <= 0
    error("Time data must be increasing and contain valid samples.");
end
fs = 1/dt;

%% ------------------------------------------------------------------------
%  MOTOR RPM
% -------------------------------------------------------------------------

rpmMask = contains(lower(vars), "rpm");
rpmNames = vars(rpmMask);

if isempty(rpmNames)
    error(["No RPM columns were found. Include names such as " ...
           "'rpm1', 'motor1_rpm', ..."]);
end

rpmNames = sortMotorColumns(rpmNames);
RPM = zeros(height(T), numel(rpmNames));

for i = 1:numel(rpmNames)
    RPM(:,i) = double(T.(rpmNames(i)));
end

omega = RPM * (2*pi/60);                    % [rad/s]
motorThrust_N = cfg.kT_N_per_rad_s2 .* omega.^2;
totalThrust_N = sum(motorThrust_N, 2);
weight_N = cfg.mass_kg * cfg.g;
thrustToWeight = totalThrust_N / weight_N;

rpmMeanAcrossMotors = mean(RPM, 2, "omitnan");
rpmSpread = max(RPM,[],2) - min(RPM,[],2);
rpmImbalance_pct = 100 * rpmSpread ./ max(rpmMeanAcrossMotors, eps);

%% ------------------------------------------------------------------------
%  POSITION / VELOCITY
% -------------------------------------------------------------------------

xName = findColumn(vars, ["x","pos_x","position_x","local_x","x_m"]);
yName = findColumn(vars, ["y","pos_y","position_y","local_y","y_m"]);
zName = findColumn(vars, ["z","pos_z","position_z","local_z","altitude","altitude_m","z_m"]);

x = getOptionalNumeric(T, xName);
y = getOptionalNumeric(T, yName);
z = getOptionalNumeric(T, zName);

if ~isempty(z) && ~cfg.localZPositiveUp
    z = -z;
end

vxName = findColumn(vars, ["vx","vel_x","velocity_x","vx_mps"]);
vyName = findColumn(vars, ["vy","vel_y","velocity_y","vy_mps"]);
vzName = findColumn(vars, ["vz","vel_z","velocity_z","vz_mps"]);

vx = getOptionalNumeric(T, vxName);
vy = getOptionalNumeric(T, vyName);
vz = getOptionalNumeric(T, vzName);

% Estimate velocities from position when velocity columns are unavailable.
if isempty(vx) && ~isempty(x)
    vx = gradient(x, t);
end
if isempty(vy) && ~isempty(y)
    vy = gradient(y, t);
end
if isempty(vz) && ~isempty(z)
    vz = gradient(z, t);
end

if ~isempty(vz) && ~cfg.localZPositiveUp
    vz = -vz;
end

%% ------------------------------------------------------------------------
%  ATTITUDE
% -------------------------------------------------------------------------

rollName  = findColumn(vars, ["roll","roll_deg","roll_rad","phi"]);
pitchName = findColumn(vars, ["pitch","pitch_deg","pitch_rad","theta"]);
yawName   = findColumn(vars, ["yaw","yaw_deg","yaw_rad","psi"]);

roll_deg = [];
pitch_deg = [];
yaw_deg = [];

if rollName ~= "" && pitchName ~= "" && yawName ~= ""
    rollRaw = double(T.(rollName));
    pitchRaw = double(T.(pitchName));
    yawRaw = double(T.(yawName));

    if cfg.attitudeInputDegrees
        roll_deg = rollRaw;
        pitch_deg = pitchRaw;
        yaw_deg = yawRaw;
    else
        roll_deg = rad2deg(rollRaw);
        pitch_deg = rad2deg(pitchRaw);
        yaw_deg = rad2deg(yawRaw);
    end
else
    % Try quaternion if Euler angles are not present.
    qxName = findColumn(vars, ["qx","quat_x","orientation_x"]);
    qyName = findColumn(vars, ["qy","quat_y","orientation_y"]);
    qzName = findColumn(vars, ["qz","quat_z","orientation_z"]);
    qwName = findColumn(vars, ["qw","quat_w","orientation_w"]);

    if qxName ~= "" && qyName ~= "" && qzName ~= "" && qwName ~= ""
        qx = double(T.(qxName));
        qy = double(T.(qyName));
        qz = double(T.(qzName));
        qw = double(T.(qwName));
        [roll_deg,pitch_deg,yaw_deg] = quaternionToEulerDeg(qx,qy,qz,qw);
    end
end

%% ------------------------------------------------------------------------
%  ANGULAR RATES
% -------------------------------------------------------------------------

pName = findColumn(vars, ["p","roll_rate","p_rad_s","angular_velocity_x"]);
qName = findColumn(vars, ["q","pitch_rate","q_rad_s","angular_velocity_y"]);
rName = findColumn(vars, ["r","yaw_rate","r_rad_s","angular_velocity_z"]);

p = getOptionalNumeric(T, pName);
q = getOptionalNumeric(T, qName);
r = getOptionalNumeric(T, rName);

%% ------------------------------------------------------------------------
%  COMMAND / SETPOINT TRACKING
% -------------------------------------------------------------------------

xCmdName = findColumn(vars, ["x_cmd","x_sp","x_setpoint","cmd_x","setpoint_x"]);
yCmdName = findColumn(vars, ["y_cmd","y_sp","y_setpoint","cmd_y","setpoint_y"]);
zCmdName = findColumn(vars, ["z_cmd","z_sp","z_setpoint","cmd_z","setpoint_z"]);

xCmd = getOptionalNumeric(T, xCmdName);
yCmd = getOptionalNumeric(T, yCmdName);
zCmd = getOptionalNumeric(T, zCmdName);

positionError_m = [];
if ~isempty(x) && ~isempty(y) && ~isempty(z) && ...
        ~isempty(xCmd) && ~isempty(yCmd) && ~isempty(zCmd)
    positionError_m = sqrt((x-xCmd).^2 + (y-yCmd).^2 + (z-zCmd).^2);
end

%% ------------------------------------------------------------------------
%  FORCE AND MOMENT ESTIMATES
% -------------------------------------------------------------------------

% Tilt-corrected vertical component of thrust.
% If no attitude data exist, total thrust is treated as vertical thrust.
if ~isempty(roll_deg) && ~isempty(pitch_deg)
    verticalThrust_N = totalThrust_N .* ...
        cosd(roll_deg) .* cosd(pitch_deg);
else
    verticalThrust_N = totalThrust_N;
end

netVerticalForce_N = verticalThrust_N - weight_N;
verticalAccelFromRPM_mps2 = netVerticalForce_N / cfg.mass_kg;

% Estimated quadrotor roll/pitch moments from individual rotor thrust.
rollMoment_Nm = nan(size(t));
pitchMoment_Nm = nan(size(t));
yawMoment_Nm = nan(size(t));

if size(motorThrust_N,2) == 4
    a = cfg.armLength_m / sqrt(2);

    % [x y] rotor locations in body frame:
    % FL, FR, RR, RL
    rotorXY = [
         a, -a
         a,  a
        -a,  a
        -a, -a
    ];

    % r x F for F = [0 0 T]:
    rollMoment_Nm  = motorThrust_N * rotorXY(:,2);
    pitchMoment_Nm = -motorThrust_N * rotorXY(:,1);

    if isfinite(cfg.kQ_Nm_per_rad_s2)
        yawDir = cfg.rotorYawDirection(:);
        if numel(yawDir) ~= 4
            error("cfg.rotorYawDirection must have four entries.");
        end
        motorYawTorque_Nm = cfg.kQ_Nm_per_rad_s2 .* omega.^2 .* yawDir.';
        yawMoment_Nm = sum(motorYawTorque_Nm,2);
    end
end

%% ------------------------------------------------------------------------
%  FLIGHT-PHASE CLASSIFICATION
% -------------------------------------------------------------------------

phase = strings(size(t));
phase(:) = "Unknown";

if ~isempty(z)
    airborne = z > cfg.groundAltitude_m;
    phase(~airborne) = "Ground";

    if ~isempty(vz)
        phase(airborne & vz > cfg.climbRate_mps) = "Climb";
        phase(airborne & vz < -cfg.climbRate_mps) = "Descent";

        if ~isempty(vx) && ~isempty(vy)
            horizontalSpeed = hypot(vx,vy);
            hoverMask = airborne & ...
                abs(vz) <= cfg.hoverVerticalRate_mps & ...
                horizontalSpeed <= cfg.hoverHorizontalSpeed_mps;

            phase(hoverMask) = "Hover";

            transitMask = airborne & phase == "Unknown";
            phase(transitMask) = "Transit";
        else
            hoverMask = airborne & abs(vz) <= cfg.hoverVerticalRate_mps;
            phase(hoverMask) = "Hover";
        end
    else
        phase(airborne) = "Airborne";
    end
end

%% ------------------------------------------------------------------------
%  WHOLE-FLIGHT STABILITY METRICS
% -------------------------------------------------------------------------

summary = struct;
summary.Duration_s = t(end)-t(1);
summary.SampleRate_Hz = fs;
summary.MeanTotalThrust_N = mean(totalThrust_N,"omitnan");
summary.StdTotalThrust_N = std(totalThrust_N,"omitnan");
summary.MeanThrustToWeight = mean(thrustToWeight,"omitnan");
summary.MinThrustToWeight = min(thrustToWeight);
summary.MaxThrustToWeight = max(thrustToWeight);
summary.MeanNetVerticalForce_N = mean(netVerticalForce_N,"omitnan");
summary.MaxRPMImbalance_pct = max(rpmImbalance_pct);
summary.MeanRPMImbalance_pct = mean(rpmImbalance_pct,"omitnan");

if ~isempty(roll_deg)
    summary.RollRMS_deg = rmsFinite(roll_deg);
    summary.RollStd_deg = std(roll_deg,"omitnan");
    summary.RollPeakAbs_deg = max(abs(roll_deg));
    summary.RollDominantFrequency_Hz = dominantFrequency(t,roll_deg);
end

if ~isempty(pitch_deg)
    summary.PitchRMS_deg = rmsFinite(pitch_deg);
    summary.PitchStd_deg = std(pitch_deg,"omitnan");
    summary.PitchPeakAbs_deg = max(abs(pitch_deg));
    summary.PitchDominantFrequency_Hz = dominantFrequency(t,pitch_deg);
end

if ~isempty(yaw_deg)
    yawUnwrapped_deg = rad2deg(unwrap(deg2rad(yaw_deg)));
    summary.YawStd_deg = std(yawUnwrapped_deg,"omitnan");
    summary.YawDominantFrequency_Hz = dominantFrequency(t,yawUnwrapped_deg);
end

if ~isempty(p)
    summary.RollRateRMS_rad_s = rmsFinite(p);
end
if ~isempty(q)
    summary.PitchRateRMS_rad_s = rmsFinite(q);
end
if ~isempty(r)
    summary.YawRateRMS_rad_s = rmsFinite(r);
end

if ~isempty(positionError_m)
    summary.PositionRMSE_m = sqrt(mean(positionError_m.^2,"omitnan"));
    summary.PositionMaxError_m = max(positionError_m);
elseif ~isempty(x) && ~isempty(y) && ~isempty(z)
    % If no command is logged, quantify drift/scatter around the mean.
    summary.PositionScatterRMS_m = sqrt(mean( ...
        (x-mean(x,"omitnan")).^2 + ...
        (y-mean(y,"omitnan")).^2 + ...
        (z-mean(z,"omitnan")).^2, "omitnan"));
end

%% ------------------------------------------------------------------------
%  USER-DEFINED TIME-WINDOW METRICS
% -------------------------------------------------------------------------

nWindows = size(cfg.windows_s,1);

if numel(cfg.windowNames) ~= nWindows
    error("cfg.windowNames must match the number of rows in cfg.windows_s.");
end

windowMetrics = table;

for k = 1:nWindows
    t0 = cfg.windows_s(k,1);
    t1 = cfg.windows_s(k,2);
    idx = t >= t0 & t <= t1;

    if nnz(idx) < 3
        warning("Window %s has fewer than three samples.", cfg.windowNames(k));
        continue;
    end

    row = table;
    row.Window = cfg.windowNames(k);
    row.Start_s = t0;
    row.End_s = t1;
    row.Samples = nnz(idx);
    row.MeanThrust_N = mean(totalThrust_N(idx),"omitnan");
    row.StdThrust_N = std(totalThrust_N(idx),"omitnan");
    row.MeanTWR = mean(thrustToWeight(idx),"omitnan");
    row.MeanNetVerticalForce_N = mean(netVerticalForce_N(idx),"omitnan");
    row.MeanRPMImbalance_pct = mean(rpmImbalance_pct(idx),"omitnan");
    row.MaxRPMImbalance_pct = max(rpmImbalance_pct(idx));

    if ~isempty(roll_deg)
        row.RollRMS_deg = rmsFinite(roll_deg(idx));
        row.RollStd_deg = std(roll_deg(idx),"omitnan");
        row.RollPeakAbs_deg = max(abs(roll_deg(idx)));
        row.RollDomFreq_Hz = dominantFrequency(t(idx),roll_deg(idx));
    end

    if ~isempty(pitch_deg)
        row.PitchRMS_deg = rmsFinite(pitch_deg(idx));
        row.PitchStd_deg = std(pitch_deg(idx),"omitnan");
        row.PitchPeakAbs_deg = max(abs(pitch_deg(idx)));
        row.PitchDomFreq_Hz = dominantFrequency(t(idx),pitch_deg(idx));
    end

    if ~isempty(positionError_m)
        row.PositionRMSE_m = sqrt(mean(positionError_m(idx).^2,"omitnan"));
        row.PositionMaxError_m = max(positionError_m(idx));
    elseif ~isempty(x) && ~isempty(y) && ~isempty(z)
        row.PositionScatterRMS_m = sqrt(mean( ...
            (x(idx)-mean(x(idx),"omitnan")).^2 + ...
            (y(idx)-mean(y(idx),"omitnan")).^2 + ...
            (z(idx)-mean(z(idx),"omitnan")).^2, "omitnan"));
    end

    if ~isempty(p)
        row.RollRateRMS_rad_s = rmsFinite(p(idx));
    end
    if ~isempty(q)
        row.PitchRateRMS_rad_s = rmsFinite(q(idx));
    end
    if ~isempty(r)
        row.YawRateRMS_rad_s = rmsFinite(r(idx));
    end

    windowMetrics = [windowMetrics; row]; %#ok<AGROW>
end

%% ------------------------------------------------------------------------
%  PHASE-BY-PHASE METRICS
% -------------------------------------------------------------------------

phaseNames = unique(phase);
phaseMetrics = table;

for k = 1:numel(phaseNames)
    thisPhase = phaseNames(k);
    if thisPhase == "Unknown"
        continue;
    end

    idx = phase == thisPhase;
    if nnz(idx) < 3
        continue;
    end

    row = table;
    row.Phase = thisPhase;
    row.Samples = nnz(idx);
    row.TimeInPhase_s = nnz(idx) * dt;
    row.MeanThrust_N = mean(totalThrust_N(idx),"omitnan");
    row.MeanTWR = mean(thrustToWeight(idx),"omitnan");
    row.MeanRPMImbalance_pct = mean(rpmImbalance_pct(idx),"omitnan");

    if ~isempty(roll_deg)
        row.RollRMS_deg = rmsFinite(roll_deg(idx));
    end
    if ~isempty(pitch_deg)
        row.PitchRMS_deg = rmsFinite(pitch_deg(idx));
    end
    if ~isempty(positionError_m)
        row.PositionRMSE_m = sqrt(mean(positionError_m(idx).^2,"omitnan"));
    end

    phaseMetrics = [phaseMetrics; row]; %#ok<AGROW>
end

%% ------------------------------------------------------------------------
%  OPTIONAL EXTERNAL WRENCH COMPARISON
% -------------------------------------------------------------------------

% If your simulation exports measured/estimated external force, use columns:
% Fx_ext, Fy_ext, Fz_ext. This section compares measured vertical force to
% the RPM-based estimate.
fxExtName = findColumn(vars, ["fx_ext","force_x","wrench_fx"]);
fyExtName = findColumn(vars, ["fy_ext","force_y","wrench_fy"]);
fzExtName = findColumn(vars, ["fz_ext","force_z","wrench_fz"]);

Fx_ext = getOptionalNumeric(T, fxExtName);
Fy_ext = getOptionalNumeric(T, fyExtName);
Fz_ext = getOptionalNumeric(T, fzExtName);

forceComparison = table;
if ~isempty(Fz_ext)
    forceComparison = table(t, Fz_ext, netVerticalForce_N, ...
        Fz_ext-netVerticalForce_N, ...
        'VariableNames', ...
        {'Time_s','MeasuredFz_N','RPMEstimatedNetVerticalForce_N','Residual_N'});
end

%% ------------------------------------------------------------------------
%  SAVE NUMERICAL OUTPUTS
% -------------------------------------------------------------------------

summaryTable = struct2table(summary);

writetable(summaryTable, fullfile(cfg.outputDir,"flight_summary.csv"));

if ~isempty(windowMetrics)
    writetable(windowMetrics, fullfile(cfg.outputDir,"window_metrics.csv"));
end

if ~isempty(phaseMetrics)
    writetable(phaseMetrics, fullfile(cfg.outputDir,"phase_metrics.csv"));
end

if ~isempty(forceComparison)
    writetable(forceComparison, fullfile(cfg.outputDir,"force_comparison.csv"));
end

%% ------------------------------------------------------------------------
%  PLOTS
% -------------------------------------------------------------------------

% 1) Trajectory / altitude
if ~isempty(x) && ~isempty(y) && ~isempty(z)
    f1 = figure("Name","Trajectory");
    plot3(x,y,z,"LineWidth",1.4);
    grid on;
    axis equal;
    xlabel("x [m]");
    ylabel("y [m]");
    zlabel("z [m]");
    title("3-D Vehicle Trajectory");
    saveas(f1, fullfile(cfg.outputDir,"trajectory_3d.png"));
end

% 2) Motor RPM and thrust
f2 = figure("Name","RPM and Thrust");
tiledlayout(3,1);

nexttile;
plot(t,RPM,"LineWidth",1.0);
grid on;
ylabel("RPM");
title("Motor RPM");
legend(rpmNames,"Interpreter","none","Location","best");

nexttile;
plot(t,motorThrust_N,"LineWidth",1.0);
grid on;
ylabel("Thrust [N]");
title("Estimated Individual Motor Thrust");

nexttile;
plot(t,totalThrust_N,"LineWidth",1.3);
hold on;
yline(weight_N,"--","Vehicle Weight");
plot(t,verticalThrust_N,"LineWidth",1.1);
grid on;
xlabel("Time [s]");
ylabel("Force [N]");
title("Total / Vertical Thrust");
legend("Total thrust","Weight","Vertical thrust","Location","best");
saveas(f2, fullfile(cfg.outputDir,"rpm_and_thrust.png"));

% 3) Stability
if ~isempty(roll_deg) || ~isempty(pitch_deg) || ~isempty(yaw_deg)
    f3 = figure("Name","Attitude Stability");
    hold on;
    if ~isempty(roll_deg), plot(t,roll_deg,"DisplayName","Roll"); end
    if ~isempty(pitch_deg), plot(t,pitch_deg,"DisplayName","Pitch"); end
    if ~isempty(yaw_deg), plot(t,yaw_deg,"DisplayName","Yaw"); end
    grid on;
    xlabel("Time [s]");
    ylabel("Angle [deg]");
    title("Attitude History");
    legend("Location","best");
    saveas(f3, fullfile(cfg.outputDir,"attitude_stability.png"));
end

% 4) Force margin and motor imbalance
f4 = figure("Name","Force Margin");
tiledlayout(2,1);

nexttile;
plot(t,netVerticalForce_N,"LineWidth",1.2);
yline(0,"--");
grid on;
ylabel("Net vertical force [N]");
title("RPM-Based Vertical Force Margin");

nexttile;
plot(t,rpmImbalance_pct,"LineWidth",1.2);
grid on;
xlabel("Time [s]");
ylabel("Motor spread [%]");
title("Motor RPM Imbalance");
saveas(f4, fullfile(cfg.outputDir,"force_margin_and_motor_balance.png"));

% 5) Position tracking
if ~isempty(positionError_m)
    f5 = figure("Name","Position Tracking");
    plot(t,positionError_m,"LineWidth",1.2);
    grid on;
    xlabel("Time [s]");
    ylabel("3-D position error [m]");
    title("Waypoint / Position Tracking Error");
    saveas(f5, fullfile(cfg.outputDir,"position_error.png"));
end

% 6) Moment estimates for a four-motor vehicle
if size(motorThrust_N,2) == 4
    f6 = figure("Name","Estimated Control Moments");
    hold on;
    plot(t,rollMoment_Nm,"DisplayName","Roll moment");
    plot(t,pitchMoment_Nm,"DisplayName","Pitch moment");
    if any(isfinite(yawMoment_Nm))
        plot(t,yawMoment_Nm,"DisplayName","Yaw moment");
    end
    grid on;
    xlabel("Time [s]");
    ylabel("Moment [N m]");
    title("Estimated Rotor-Induced Body Moments");
    legend("Location","best");
    saveas(f6, fullfile(cfg.outputDir,"estimated_moments.png"));
end

% 7) Attitude oscillation spectra
if ~isempty(roll_deg)
    saveSpectrumFigure(t,roll_deg,"Roll", ...
        fullfile(cfg.outputDir,"roll_spectrum.png"));
end
if ~isempty(pitch_deg)
    saveSpectrumFigure(t,pitch_deg,"Pitch", ...
        fullfile(cfg.outputDir,"pitch_spectrum.png"));
end

%% ------------------------------------------------------------------------
%  RETURN RESULTS
% -------------------------------------------------------------------------

results = struct;
results.config = cfg;
results.time_s = t;
results.rpm = RPM;
results.omega_rad_s = omega;
results.motorThrust_N = motorThrust_N;
results.totalThrust_N = totalThrust_N;
results.verticalThrust_N = verticalThrust_N;
results.netVerticalForce_N = netVerticalForce_N;
results.verticalAccelFromRPM_mps2 = verticalAccelFromRPM_mps2;
results.thrustToWeight = thrustToWeight;
results.rpmImbalance_pct = rpmImbalance_pct;
results.rollMoment_Nm = rollMoment_Nm;
results.pitchMoment_Nm = pitchMoment_Nm;
results.yawMoment_Nm = yawMoment_Nm;
results.phase = phase;
results.summary = summaryTable;
results.windowMetrics = windowMetrics;
results.phaseMetrics = phaseMetrics;
results.forceComparison = forceComparison;

fprintf("\nRISE flight analysis complete.\n");
fprintf("Results saved to: %s\n\n", cfg.outputDir);
disp(summaryTable);

end

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function name = findColumn(vars, aliases)
% Exact, case-insensitive column-name matching.
name = "";
varsLower = lower(strtrim(vars));
for k = 1:numel(aliases)
    idx = find(varsLower == lower(strtrim(aliases(k))),1);
    if ~isempty(idx)
        name = vars(idx);
        return;
    end
end
end

function x = getOptionalNumeric(T, name)
if name == ""
    x = [];
else
    x = double(T.(name));
end
end

function t = normalizeTime(tRaw)
if isdatetime(tRaw)
    t = seconds(tRaw - tRaw(1));
elseif isduration(tRaw)
    t = seconds(tRaw - tRaw(1));
else
    t = double(tRaw);
    t = t - t(1);
end
t = t(:);
end

function rpmNames = sortMotorColumns(rpmNames)
% Sort motor RPM columns by the first number appearing in each name.
key = inf(size(rpmNames));
for k = 1:numel(rpmNames)
    token = regexp(rpmNames(k), '\d+', 'match', 'once');
    if ~isempty(token)
        key(k) = str2double(token);
    end
end
[~,idx] = sort(key);
rpmNames = rpmNames(idx);
end

function value = rmsFinite(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = sqrt(mean(x.^2));
end
end

function fDom = dominantFrequency(t,x)
% Toolbox-free dominant-frequency estimate using the FFT.
valid = isfinite(t) & isfinite(x);
t = t(valid);
x = x(valid);

if numel(x) < 8
    fDom = NaN;
    return;
end

x = x - mean(x);
dt = median(diff(t));

if ~isfinite(dt) || dt <= 0
    fDom = NaN;
    return;
end

N = numel(x);
Y = fft(x);
P2 = abs(Y/N);
halfN = floor(N/2) + 1;
P1 = P2(1:halfN);

f = (0:halfN-1)'/(N*dt);

% Ignore DC component.
if numel(P1) <= 1
    fDom = NaN;
    return;
end

[~,idx] = max(P1(2:end));
idx = idx + 1;
fDom = f(idx);
end

function saveSpectrumFigure(t,x,labelText,fileName)
valid = isfinite(t) & isfinite(x);
t = t(valid);
x = x(valid);

if numel(x) < 8
    return;
end

x = x - mean(x);
dt = median(diff(t));
N = numel(x);

Y = fft(x);
P2 = abs(Y/N);
halfN = floor(N/2) + 1;
P1 = P2(1:halfN);
f = (0:halfN-1)'/(N*dt);

fig = figure("Name",labelText + " Spectrum");
plot(f,P1,"LineWidth",1.2);
grid on;
xlabel("Frequency [Hz]");
ylabel("Amplitude");
title(labelText + " Oscillation Spectrum");
xlim([0, min(0.5/dt, 10)]);
saveas(fig,fileName);
end

function [roll_deg,pitch_deg,yaw_deg] = quaternionToEulerDeg(qx,qy,qz,qw)
% Quaternion [x y z w] to ZYX Euler angles.
% Returns roll, pitch, yaw in degrees.

sinr_cosp = 2 .* (qw.*qx + qy.*qz);
cosr_cosp = 1 - 2 .* (qx.^2 + qy.^2);
roll = atan2(sinr_cosp, cosr_cosp);

sinp = 2 .* (qw.*qy - qz.*qx);
sinp = max(-1,min(1,sinp));
pitch = asin(sinp);

siny_cosp = 2 .* (qw.*qz + qx.*qy);
cosy_cosp = 1 - 2 .* (qy.^2 + qz.^2);
yaw = atan2(siny_cosp, cosy_cosp);

roll_deg = rad2deg(roll);
pitch_deg = rad2deg(pitch);
yaw_deg = rad2deg(yaw);
end
