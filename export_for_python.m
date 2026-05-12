% EXPORT_FOR_PYTHON
% Converts MATLAB timetable .mat files into plain-array .mat files (v7)
% that Python/scipy can load without issues.
%
% Run this once per sensor file. Output: Sensor_Back_<horse>_exported.mat

horses = ["Albin", "Baldur", "Sigge", "Sinus", "Sam"];

for h = 1:numel(horses)
    horse = horses(h);
    inFile  = "Sensor_Back_" + horse + ".mat";
    outFile = "Sensor_Back_" + horse + "_exported.mat";

    if ~isfile(inFile)
        fprintf("Skipping %s (not found)\n", inFile);
        continue;
    end

    fprintf("Processing %s ...\n", inFile);
    S = load(inFile);

    out = struct();

    % --- Acceleration (Nx3, m/s²) ---
    if isfield(S, 'Acceleration') && ~isempty(S.Acceleration)
        [t, v] = tt2arrays(S.Acceleration);
        out.acc_time = t;
        out.acc_xyz  = v;
        fprintf("  Acceleration: %d samples, %.1f s\n", length(t), t(end)-t(1));
    end

    % --- Orientation (Nx3, degrees: Azimuth, Pitch, Roll) ---
    if isfield(S, 'Orientation') && ~isempty(S.Orientation)
        [t, v] = tt2arrays(S.Orientation);
        out.ori_time = t;
        out.ori_xyz  = v;
        fprintf("  Orientation:  %d samples, %.1f s\n", length(t), t(end)-t(1));
    end

    % --- Angular Velocity (Nx3, rad/s) ---
    if isfield(S, 'AngularVelocity') && ~isempty(S.AngularVelocity)
        [t, v] = tt2arrays(S.AngularVelocity);
        out.ang_time = t;
        out.ang_xyz  = v;
        fprintf("  AngularVel:   %d samples, %.1f s\n", length(t), t(end)-t(1));
    end

    % --- Magnetic Field (Nx3, µT) ---
    if isfield(S, 'MagneticField') && ~isempty(S.MagneticField)
        [t, v] = tt2arrays(S.MagneticField);
        out.mag_time = t;
        out.mag_xyz  = v;
        fprintf("  MagneticField:%d samples, %.1f s\n", length(t), t(end)-t(1));
    end

    % --- Position (includes speed, lat, lon, etc.) ---
    if isfield(S, 'Position') && ~isempty(S.Position)
        [t, v] = tt2arrays(S.Position);
        out.pos_time = t;
        out.pos_data = v;
        out.pos_columns = string(S.Position.Properties.VariableNames);
        fprintf("  Position:     %d samples, %.1f s, cols: %s\n", ...
            length(t), t(end)-t(1), strjoin(out.pos_columns, ", "));
    end

    % --- Column names for the 3-column sensors ---
    if isfield(S, 'Acceleration')
        out.acc_columns = string(S.Acceleration.Properties.VariableNames);
    end
    if isfield(S, 'Orientation')
        out.ori_columns = string(S.Orientation.Properties.VariableNames);
    end
    if isfield(S, 'AngularVelocity')
        out.ang_columns = string(S.AngularVelocity.Properties.VariableNames);
    end
    if isfield(S, 'MagneticField')
        out.mag_columns = string(S.MagneticField.Properties.VariableNames);
    end

    % --- Save as v7 (no timetable objects, just plain arrays) ---
    save(outFile, '-struct', 'out', '-v7');
    fprintf("  Saved: %s\n\n", outFile);
end

disp('Done. Copy the *_exported.mat files to your Python project folder.');

% Backup save as csv if the mat does not work:



% =========================================================================
function [t_sec, values] = tt2arrays(TT)
    % Convert a timetable to seconds-from-start vector + data matrix
    rt = TT.Properties.RowTimes;
    t_sec = seconds(rt - rt(1));
    values = TT.Variables;
    % Ensure double
    t_sec  = double(t_sec);
    values = double(values);
end
