clear; clc; close all;

% =====================================================
% MASTER SCRIPT
% Choose:
%   - horse (_Albin / _Baldur)
%   - mode  ("full" or "segments")
%   - optional FFT filter settings
% =====================================================

folder = "F:\media\code\matlabDrive\MobileSensorData\albinBaldur"; %change to your folder

% ---- Pick horse ----
%horse = "_Albin";
horse = "_Baldur";

% ---- Pick mode ----
% segments gives FFT at every straight section
% full gives entire dataset
%mode = "segments"; 
%mode = "full";
mode = "kalman"


names = ["Back","Left","Right"];

% ---- clap markers ----
marker_Albin  = 260.22;
marker_Baldur = 241.74;

if horse == "_Albin"
    markerTime = marker_Albin;
else
    markerTime = marker_Baldur;
end

% ---- FFT filter  ----
useFilter = false;   % use filter before FFT
fHP = 0.5;           % Hz
fLP = 20;            % Hz

% ---- Straight sections (seconds) ----
windows_Baldur = [ ...
    120 125;
    135 141;
    151 157;
    167 171;
    181 187;
    196 203;
    211 219];

windows_Albin = [ ...
    126 133;
    143 149;
    160 167;
    177 185;
    194 201;
    210 219;
    228 234];

if horse == "_Albin"
    windows = windows_Albin;
else
    windows = windows_Baldur;
end

% =====================================================
% MAIN
% =====================================================

for k = 1:numel(names)
    tag = names(k);

    matPath = fullfile(folder, "Sensor_" + tag + horse + ".mat");
    m4aPath = fullfile(folder, "Sensor_" + tag + horse + ".m4a");

    figTitle = "Sensor " + tag + " " + horse;

    if mode == "full"
        plot_full_overview(matPath, m4aPath, figTitle, markerTime);
    elseif mode == "segments"
        plot_segments_and_fft(matPath, figTitle, windows, useFilter, fHP, fLP);
    elseif mode == "kalman"
        plot_kalman_segments(matPath,windows,figTitle);
    else
        % do nothing
    end
    
end

disp('Done.');

% =====================================================
% FULL
% =====================================================
function plot_full_overview(matPath, m4aPath, figTitle, markerTime)

fprintf("\nMAT: %s\n", matPath);
fprintf("M4A: %s\n", m4aPath);

if ~isfile(matPath)
    warning("MAT file not found: %s", matPath);
    return;
end

S = load(matPath);

AccTT = S.Acceleration;
OriTT = S.Orientation;
AngTT = S.AngularVelocity;
MagTT = S.MagneticField;
PosTT = S.Position;
AudioInfo = S.AudioInfo;

[t_acc, acc] = tt_to_seconds_and_vars(AccTT);
[t_ori, ori] = tt_to_seconds_and_vars(OriTT);
[t_ang, ang] = tt_to_seconds_and_vars(AngTT);
[t_mag, mag] = tt_to_seconds_and_vars(MagTT);

acc = ensureNx3(acc);
ori = ensureNx3(ori);
ang = ensureNx3(ang);
mag = ensureNx3(mag);

% Load audio
audio = [];
t_audio = [];
Fs_audio = [];
if isfile(m4aPath)
    [audio, Fs_audio] = audioread(m4aPath);
    if size(audio,2) > 1, audio = mean(audio,2); end
    t_audio = (0:length(audio)-1)' / Fs_audio;
else
    warning("Audio file not found: %s", m4aPath);
end

f = figure('Name', figTitle + " | Full", 'Color','w');
tiledlayout(6,1,'Padding','compact','TileSpacing','compact');

% 1) Acceleration
nexttile; hold on;
plot(t_acc, acc(:,1), 'r', 'DisplayName','X');
plot(t_acc, acc(:,2), 'g', 'DisplayName','Y');
plot(t_acc, acc(:,3), 'b', 'DisplayName','Z');
xline(markerTime, '--k', sprintf('%.2f s', markerTime));
title('Acceleration'); ylabel('m/s^2'); grid on; legend('show','Location','best');
hold off;

% 2) Orientation
nexttile; hold on;
plot(t_ori, ori(:,1), 'c', 'DisplayName','Azimuth');
plot(t_ori, ori(:,2), 'm', 'DisplayName','Pitch');
plot(t_ori, ori(:,3), 'k', 'DisplayName','Roll');
xline(markerTime, '--k', sprintf('%.2f s', markerTime));
title('Orientation'); ylabel('deg'); grid on; legend('show','Location','best');
hold off;

% 3) Angular Velocity
nexttile; hold on;
plot(t_ang, ang(:,1), 'r', 'DisplayName','X');
plot(t_ang, ang(:,2), 'g', 'DisplayName','Y');
plot(t_ang, ang(:,3), 'b', 'DisplayName','Z');
xline(markerTime, '--k', sprintf('%.2f s', markerTime));
title('Angular Velocity'); ylabel('rad/s'); grid on; legend('show','Location','best');
hold off;

% 4) Magnetic Field
nexttile; hold on;
plot(t_mag, mag(:,1), 'r', 'DisplayName','X');
plot(t_mag, mag(:,2), 'g', 'DisplayName','Y');
plot(t_mag, mag(:,3), 'b', 'DisplayName','Z');
xline(markerTime, '--k', sprintf('%.2f s', markerTime));
title('Magnetic Field'); ylabel('\muT'); grid on; legend('show','Location','best');
hold off;

% 5) Speed
nexttile; hold on;
if ~isempty(PosTT) && any(strcmpi(PosTT.Properties.VariableNames, "speed"))
    t_pos = seconds(PosTT.Properties.RowTimes - PosTT.Properties.RowTimes(1));
    speed = PosTT{:, strcmpi(PosTT.Properties.VariableNames, "speed")};
    plot(t_pos, speed, 'b', 'DisplayName','speed');
    xline(markerTime, '--k', sprintf('%.2f s', markerTime));
    title('Speed'); ylabel('m/s'); grid on; legend('show','Location','best');
else
    title('Speed (not available)'); grid on;
end
hold off;

% 6) Audio
nexttile; hold on;
if ~isempty(audio)
    plot(t_audio, audio, 'Color', [0.3 0.3 0.3]);
    xline(markerTime, '--k', sprintf('%.2f s', markerTime));
    if isfield(AudioInfo,'SampleRate')
        title(sprintf('Audio (Fs = %g Hz)', AudioInfo.SampleRate));
    else
        title(sprintf('Audio (Fs = %d Hz)', Fs_audio));
    end
    ylabel('Amplitude'); grid on;
else
    title('Audio (not available)'); grid on;
end
xlabel('Time (s)');
hold off;

ax = findobj(f,'Type','axes');
linkaxes(ax,'x');

end

% =====================================================
% SEGMENTS
% =====================================================
function plot_segments_and_fft(matPath, figTitle, windows, useFilter, fHP, fLP)

fprintf("\nMAT: %s\n", matPath);

if ~isfile(matPath)
    warning("MAT file not found: %s", matPath);
    return;
end

S = load(matPath);

AccTT = S.Acceleration;
OriTT = S.Orientation;
AngTT = S.AngularVelocity;

[t_acc, acc] = tt_to_seconds_and_vars(AccTT);
[t_ori, ori] = tt_to_seconds_and_vars(OriTT);
[t_ang, ang] = tt_to_seconds_and_vars(AngTT);


acc = ensureNx3(acc);
ori = ensureNx3(ori);
ang = ensureNx3(ang);

nW = size(windows,1);

% ---- TIME DOMAIN GRID ----
f1 = figure('Color','w', 'Name', figTitle + " | Segments (Time)");
tl1 = tiledlayout(f1, nW, 3, 'Padding','compact','TileSpacing','compact');
title(tl1, figTitle + " | Time-domain segments");

for i = 1:nW
    t0 = windows(i,1); t1 = windows(i,2);

    nexttile(tl1, (i-1)*3 + 1); hold on;
    idx = (t_acc >= t0) & (t_acc <= t1);
    if any(idx)
        plot(t_acc(idx), acc(idx,1), 'DisplayName','X');
        plot(t_acc(idx), acc(idx,2), 'DisplayName','Y');
        plot(t_acc(idx), acc(idx,3), 'DisplayName','Z');
        grid on; xlim([t0 t1]); ylabel('m/s^2');
    else
        grid on; xlim([t0 t1]);
        text(0.5,0.5,'No data','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('Acceleration'); legend('show','Location','best'); end
    if i == nW, xlabel('Time (s)'); end
    hold off;

    nexttile(tl1, (i-1)*3 + 2); hold on;
    idx = (t_ori >= t0) & (t_ori <= t1);
    if any(idx)
        plot(t_ori(idx), ori(idx,1), 'DisplayName','Az');
        plot(t_ori(idx), ori(idx,2), 'DisplayName','Pitch');
        plot(t_ori(idx), ori(idx,3), 'DisplayName','Roll');
        grid on; xlim([t0 t1]); ylabel('deg');
    else
        grid on; xlim([t0 t1]);
        text(0.5,0.5,'No data','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('Orientation'); legend('show','Location','best'); end
    if i == nW, xlabel('Time (s)'); end
    hold off;

    nexttile(tl1, (i-1)*3 + 3); hold on;
    idx = (t_ang >= t0) & (t_ang <= t1);
    if any(idx)
        plot(t_ang(idx), ang(idx,1), 'DisplayName','X');
        plot(t_ang(idx), ang(idx,2), 'DisplayName','Y');
        plot(t_ang(idx), ang(idx,3), 'DisplayName','Z');
        grid on; xlim([t0 t1]); ylabel('rad/s');
    else
        grid on; xlim([t0 t1]);
        text(0.5,0.5,'No data','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('Angular Velocity'); legend('show','Location','best'); end
    if i == nW, xlabel('Time (s)'); end
    hold off;
end

% ---- FFT GRID ----
f2 = figure('Color','w', 'Name', figTitle + " | Segments (FFT)");
tl2 = tiledlayout(f2, nW, 3, 'Padding','compact','TileSpacing','compact');

if useFilter
    title(tl2, sprintf('%s | FFT (band-pass %.1f–%.1f Hz)', figTitle, fHP, fLP));
else
    title(tl2, sprintf('%s | FFT (no filter)', figTitle));
end

for i = 1:nW
    t0 = windows(i,1); t1 = windows(i,2);

    nexttile(tl2, (i-1)*3 + 1); hold on;

    [~, tt, xx] = extract_window(t_acc, acc, t0, t1);

    %fprintf("Segment %.1f–%.1f s\n",t0,t1);
    %analyse_phase_xyz(tt,xx);

    %fprintf("Segment %.1f–%.1f s\n",t0,t1);
    %analyse_cross_phase_xyz(tt,xx);


    if numel(tt) >= 8
        [ff, P1, Fs] = fft_xyz(tt, xx, useFilter, fHP, fLP);
        plot(ff, P1(:,1), 'DisplayName','X');
        plot(ff, P1(:,2), 'DisplayName','Y');
        plot(ff, P1(:,3), 'DisplayName','Z');
        grid on; xlim([0, min(50, Fs/2)]); ylabel('|A(f)|');
    else
        grid on;
        text(0.5,0.5,'Not enough samples','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('FFT Acceleration'); legend('show','Location','best'); end
    if i == nW, xlabel('Frequency (Hz)'); end
    hold off;

    nexttile(tl2, (i-1)*3 + 2); hold on;
    [~, tt, xx] = extract_window(t_ori, ori, t0, t1);
    if numel(tt) >= 8
        [ff, P1, Fs] = fft_xyz(tt, xx, useFilter, fHP, fLP);
        plot(ff, P1(:,1), 'DisplayName','Az');
        plot(ff, P1(:,2), 'DisplayName','Pitch');
        plot(ff, P1(:,3), 'DisplayName','Roll');
        grid on; xlim([0, min(50, Fs/2)]); ylabel('|A(f)|');
    else
        grid on;
        text(0.5,0.5,'Not enough samples','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('FFT Orientation'); legend('show','Location','best'); end
    if i == nW, xlabel('Frequency (Hz)'); end
    hold off;

    nexttile(tl2, (i-1)*3 + 3); hold on;
    [~, tt, xx] = extract_window(t_ang, ang, t0, t1);
    if numel(tt) >= 8
        [ff, P1, Fs] = fft_xyz(tt, xx, useFilter, fHP, fLP);
        plot(ff, P1(:,1), 'DisplayName','X');
        plot(ff, P1(:,2), 'DisplayName','Y');
        plot(ff, P1(:,3), 'DisplayName','Z');
        grid on; xlim([0, min(50, Fs/2)]); ylabel('|A(f)|');
    else
        grid on;
        text(0.5,0.5,'Not enough samples','Units','normalized','HorizontalAlignment','center');
    end
    if i == 1, title('FFT Angular Velocity'); legend('show','Location','best'); end
    if i == nW, xlabel('Frequency (Hz)'); end
    hold off;
end

end
% =======================
% sensor fusion test
% =======================

function plot_kalman_segments(matPath, windows, titlefigure)

S = load(matPath);
[t_acc, acc] = tt_to_seconds_and_vars(S.Acceleration);
acc = ensureNx3(acc);

figure('Name', titlefigure, 'Color', 'w');

for i = 1:size(windows,1)

    t0 = windows(i,1);
    t1 = windows(i,2);

    [~, tt, xx] = extract_window(t_acc, acc, t0, t1);

    if numel(tt) < 10
        continue;
    end
    
    % %estimate gravity direction and remove it
    g = mean(xx,1); 
    xx = xx - g;

      % Kalman
    [pos, vel] = reconstruct_translation_kalman(tt, xx);

    % FFT on velocity
    [ff, P1, Fs] = fft_xyz(tt, vel, false, 2.5, 5);

    % Plot
    % FFT on velocity
    [ff, P1, Fs] = fft_xyz(tt, vel, true, 2.5, 6);

    % Plot
    subplot(size(windows,1),3,(i-1)*3+1)
    plot(ff, P1(:,1)); grid on;
    title(sprintf("Seg %d Surge",i))
    xlim([0 10])

    subplot(size(windows,1),3,(i-1)*3+2)
    plot(ff, P1(:,2)); grid on;
    title("Sway")
    xlim([0 10])

    subplot(size(windows,1),3,(i-1)*3+3)
    plot(ff, P1(:,3)); grid on;
    title("Heave")
    xlim([0 10])

    % ---- Plot ----
    % subplot(size(windows,1),3,(i-1)*3+1)
    % plot(tt, pos(:,1)); grid on;
    % title(sprintf("Seg %d Surge",i))
    % 
    % subplot(size(windows,1),3,(i-1)*3+2)
    % plot(tt, pos(:,2)); grid on;
    % title("Sway")
    % 
    % subplot(size(windows,1),3,(i-1)*3+3)
    % plot(tt, pos(:,3)); grid on;
    % title("Heave")

end

end

%% =====================================================
% 🧠 KALMAN RECONSTRUCTION
% =====================================================
function [pos, vel] = reconstruct_translation_kalman(t, acc)

dt = median(diff(t));
N = length(t);

pos = zeros(N,3);
vel = zeros(N,3);

for axis = 1:3
    [pos(:,axis), vel(:,axis)] = kalman_1D(acc(:,axis), dt);
end

end

function [pos, vel] = kalman_1D(acc, dt)

N = length(acc);

x = [0;0]; % [pos; vel]

A = [1 dt; 0 1];
B = [0.5*dt^2; dt];
C = [1 0];

Q = [1e-4 0; 0 1e-2];
R = 1e-2;

P = eye(2);

pos = zeros(N,1);
vel = zeros(N,1);

for k = 1:N

    % Predict
    x = A*x + B*acc(k);
    P = A*P*A' + Q;

    % Stabilization (prevents drift)
    z = 0;

    % Update
    K = P*C' / (C*P*C' + R);
    x = x + K*(z - C*x);
    P = (eye(2)-K*C)*P;

    pos(k) = x(1);
    vel(k) = x(2);
end

end

function [tsec, X] = tt_to_seconds_and_vars(TT)
    if isempty(TT)
        tsec = [];
        X = [];
        return;
    end
    rt = TT.Properties.RowTimes;
    tsec = seconds(rt - rt(1));
    X = TT.Variables;
end

function X = ensureNx3(X)
    if isempty(X), return; end
    if size(X,2) == 1
        X = [X, nan(size(X)), nan(size(X))];
    elseif size(X,2) == 2
        X = [X, nan(size(X,1),1)];
    elseif size(X,2) > 3
        X = X(:,1:3);
    end
end

function [idx, t_win, x_win] = extract_window(t, X, t0, t1)
    idx = (t >= t0) & (t <= t1);
    t_win = t(idx);
    x_win = X(idx,:);
end

function [f, P1, Fs] = fft_xyz(t, X, useFilter, fHP, fLP)

t = t(:);
X = X(:,1:3);

dt = median(diff(t));
Fs = 1/dt;

t_u = (t(1):dt:t(end))';
X_u = interp1(t, X, t_u, 'linear', 'extrap');

X_u = detrend(X_u);

if useFilter
    nyq = Fs/2;
    hp = max(0.01, min(fHP, nyq*0.99));
    lp = max(hp*1.1, min(fLP, nyq*0.99));
    [b,a] = butter(4, [hp lp]/nyq, 'bandpass');
    X_u = filtfilt(b,a,X_u);
end

N = size(X_u,1);
w = hann(N);
Xw = X_u .* w;

Y = fft(Xw);
P2 = abs(Y/N);

P1 = P2(1:floor(N/2)+1, :);
P1(2:end-1,:) = 2*P1(2:end-1,:);

f = Fs*(0:floor(N/2))'/N;

end

function analyse_phase_xyz(t, X)
if isempty(t) || isempty(X) || size(X,1) < 2
    fprintf("Warning: Insufficient data for phase analysis\n\n");
    return;
end

t = t(:);
X = X(:,1:3);

dt = median(diff(t));
Fs = 1/dt;

% Resample uniform
t_u = (t(1):dt:t(end))';
X_u = interp1(t, X, t_u, 'linear', 'extrap');

% Remove trends
X_u = detrend(X_u);

N = size(X_u,1);
w = hann(N);
Xw = X_u .* w;

Y = fft(Xw);

f = Fs*(0:floor(N/2))'/N;
Y = Y(1:floor(N/2)+1,:);

% Find dominant frequency from total energy
P = abs(Y);
Psum = sum(P,2);

[~,idx] = max(Psum(2:end)); % ignore DC
idx = idx + 1;

f_dom = f(idx);

% Phase angles
phaseX = angle(Y(idx,1));
phaseY = angle(Y(idx,2));
phaseZ = angle(Y(idx,3));

% Convert to degrees
phaseX = rad2deg(phaseX);
phaseY = rad2deg(phaseY);
phaseZ = rad2deg(phaseZ);

fprintf("Dominant frequency: %.2f Hz\n", f_dom);
fprintf("Phase X: %.1f°\n", phaseX);
fprintf("Phase Y: %.1f°\n", phaseY);
fprintf("Phase Z: %.1f°\n", phaseZ);

fprintf("Phase difference X-Y: %.1f°\n", wrapTo180(phaseX-phaseY));
fprintf("Phase difference X-Z: %.1f°\n", wrapTo180(phaseX-phaseZ));
fprintf("Phase difference Y-Z: %.1f°\n\n", wrapTo180(phaseY-phaseZ));

end
function analyse_cross_phase_xyz(t, X)

% Input validation
if isempty(t) || isempty(X) || size(X,1) < 2
    fprintf("Warning: Insufficient data for cross-phase analysis\n\n");
    return;
end

t = t(:);
X = X(:,1:3);

% Validate that t has enough points
if length(t) < 2
    fprintf("Warning: Time vector too short (< 2 samples)\n\n");
    return;
end

dt = median(diff(t));

% Check for invalid dt
if dt <= 0 || isnan(dt) || isinf(dt)
    fprintf("Warning: Invalid sampling interval\n\n");
    return;
end

Fs = 1/dt;

% Resample uniformly - handle edge cases
t_start = t(1);
t_end = t(end);

if t_start >= t_end
    fprintf("Warning: Invalid time range\n\n");
    return;
end

t_u = (t_start:dt:t_end)';

% If resampling produces empty array, fallback to original data
if isempty(t_u)
    t_u = t;
end

X_u = interp1(t, X, t_u, 'linear', 'extrap');

% Remove trends
X_u = detrend(X_u);

x = X_u(:,1);
y = X_u(:,2);
z = X_u(:,3);

% Adaptive Welch parameters based on signal length
N_signal = length(x);

% Need at least 2 samples for cpsd
if N_signal < 2
    fprintf("Warning: Resampled signal too short\n\n");
    return;
end

nfft = min(1024, 2^nextpow2(N_signal));
window_len = min(512, floor(N_signal / 4));
noverlap = floor(window_len / 2);

% Ensure window_len is at least a few samples
if window_len < 16
    window_len = min(16, N_signal);
    noverlap = max(0, floor(window_len / 2));
end

% Additional safety: ensure window is not larger than signal
if window_len >= N_signal
    window_len = max(2, floor(N_signal / 2));
    noverlap = max(0, floor(window_len / 2));
end

window = hanning(window_len);

% Cross spectra with error handling
try
    [Pxy,f] = cpsd(x,y,window,noverlap,nfft,Fs);
    [Pxz,~] = cpsd(x,z,window,noverlap,nfft,Fs);
    [Pyz,~] = cpsd(y,z,window,noverlap,nfft,Fs);
catch ME
    fprintf("Warning: Could not compute cross-spectra (%s)\n\n", ME.message);
    return;
end

% Coherence
try
    Cxy = mscohere(x,y,window,noverlap,nfft,Fs);
    Cxz = mscohere(x,z,window,noverlap,nfft,Fs);
    Cyz = mscohere(y,z,window,noverlap,nfft,Fs);
catch ME
    fprintf("Warning: Could not compute coherence (%s)\n\n", ME.message);
    return;
end

% ---- stride frequency band ----
band = (f > 2 & f < 5);

bandIdx = find(band);

if isempty(bandIdx)
    fprintf("Warning: No data in stride frequency band (2-5 Hz)\n\n");
    return;
end

% find strongest peak inside band
[~,localIdx] = max(abs(Pxy(band)));

idx = bandIdx(localIdx);

f_dom = f(idx);

phaseXY = rad2deg(angle(Pxy(idx)));
phaseXZ = rad2deg(angle(Pxz(idx)));
phaseYZ = rad2deg(angle(Pyz(idx)));

fprintf("Dominant stride frequency: %.2f Hz\n",f_dom);
fprintf("Phase X→Y: %.1f° (coh %.2f)\n",phaseXY,Cxy(idx));
fprintf("Phase X→Z: %.1f° (coh %.2f)\n",phaseXZ,Cxz(idx));
fprintf("Phase Y→Z: %.1f° (coh %.2f)\n\n",phaseYZ,Cyz(idx));

end
