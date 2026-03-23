clc; clear; close all;

%% -------------------- Connection --------------------
server_ip   = '127.0.0.1';
server_port = 55001;
client = tcpclient(server_ip, server_port, "Timeout", 20);
fprintf(1, "Connected to Blender server\n");

%% -------------------- Render Params --------------------
width  = 1080;
height = 720;

%% -------------------- Camera / Scene Setup --------------------
B = 1.0;     % baseline in meters

camName = "Camera";

camX0 = 0;
camY0 = -20.0;
camZ0 = 4.7;

pitch = 90;
roll  = 0;
yaw   = 0;   % if wrong direction, try -90

ballName = "tennisBall";
ballX = camX0;
ballZ = 5.0;

%% -------------------- Stereo Intrinsics from Blender --------------------
% From your Blender screenshot:
% focal length = 43 mm, sensor width = 36 mm
f_mm       = 43;
sensorW_mm = 36;

% Convert focal length to pixels
f_px = (f_mm / sensorW_mm) * width;

cx = width/2;
cy = height/2;

fprintf("\n---- Using stereo params ----\n");
fprintf("B = %.3f m\n", B);
fprintf("f = %.1f mm, sensorW = %.1f mm, f_px = %.1f px\n", f_mm, sensorW_mm, f_px);
fprintf("Resolution = %dx%d, cx=%.1f cy=%.1f\n\n", width, height, cx, cy);

%% -------------------- Sweep (Ball moves only in Y) --------------------
ballY_vals = 3 : 0.5 : 5;   % meters in front of world origin (your choice)
N = numel(ballY_vals);

Y_est_all = nan(N,1);   % estimated forward depth
X_est_all = nan(N,1);   % estimated horizontal position
Z_est_all = nan(N,1);   % estimated vertical position

Z_true = nan(N,1);      % true forward depth
Z_err  = nan(N,1);      % forward depth error

disp_px = nan(N,1);
ok = false(N,1);

showDebug = true;

% If MATLAB is hiding figure windows in your environment, force normal style:
set(0,'DefaultFigureWindowStyle','normal');

for i = 1:N
    ballY = ballY_vals(i);

    % 1) Move the ball: ONLY Y changes
    blenderLink(client, width, height, ballX, ballY, ballZ, 0, 0, 0, ballName);

    % 2) Render Left/Right by shifting camera along BASELINE axis (world X)
    imgL = blenderLink(client, width, height, ...
        camX0 - B/2, camY0, camZ0, pitch, roll, yaw, camName);

    imgR = blenderLink(client, width, height, ...
        camX0 + B/2, camY0, camZ0, pitch, roll, yaw, camName);

    % 3) Detect ball centers
    [cLx, cLy] = detectBallCircle(imgL);
    [cRx, cRy] = detectBallCircle(imgR);

    if any(isnan([cLx cLy cRx cRy]))
        fprintf("ballY=%.2f -> detection FAILED\n", ballY);
        continue;
    end

    % 4) Horizontal disparity in pixels (must be X if baseline is X)
    dx = (cLx - cRx);
    disp_px(i) = abs(dx);

    if disp_px(i) < 1
        fprintf("ballY=%.2f -> disparity too small (%.3f px)\n", ballY, disp_px(i));
        continue;
    end

    ok(i) = true;

    % 5) Depth estimate (meters)
    % Z = (B * f_px) / disparity_px
    scale = 1.166; %calibration for blender scale
    % Z_est(i) = 1.166 * (B * f_px) / disp_px(i);
    % 
    % X_est = ((cLx - cx) * Y_est) / f_px;
    % Z_est_world = camZ0 - ((cLy - cy) * Y_est) / f_px;
    % Y_world = camY0 + Y_est;
    % X_world = camX0 + X_est;

    Y_est = scale * (B * f_px) / disp_px(i);           % depth
    X_est = ((cLx - cx) * Y_est) / f_px - B/2;         % left/right (subract half baseline to accommodate for displacement)
    Z_est = camZ0 - ((cLy - cy) * Y_est) / f_px;       % height

    Y_est_all(i) = Y_est;
    X_est_all(i) = X_est;
    Z_est_all(i) = Z_est;

    % 6) True depth: if camera looks along +Y, depth is (ballY - camY0)
    % Z_true(i) = ballY - camY0;
    % 
    % Z_err(i) = abs(Z_est(i) - Z_true(i));

    Z_true(i) = ballY - camY0;      % true forward distance
    Z_err(i)  = abs(Y_est - Z_true(i));

    % fprintf("ballY=%.2f  Ztrue=%.2f  disp=%.2f px  Zest=%.3f  err=%.3f\n", ...
    %     ballY, Z_true(i), disp_px(i), Z_est(i), Z_err(i));

    fprintf("ballY=%.2f m, disparity=%.2f px, Depth_est=%.3f m, Depth_true=%.3f m, Error=%.3f m, X=%.3f m, Z=%.3f m\n", ...
          ballY, disp_px(i), Y_est, Z_true(i), Z_err(i), X_est, Z_est);

    if showDebug
        figure(1); clf;
        set(gcf, 'Position', get(0,'Screensize'));

        subplot(1,2,1); imshow(imgL);
        title(sprintf("Left (ballY=%.2f)", ballY)); axis off; hold on;
        drawCrosshair(gca, cLx, cLy);

        subplot(1,2,2); imshow(imgR);
        title(sprintf("Right (dx=%.2f px)", disp_px(i))); axis off; hold on;
        drawCrosshair(gca, cRx, cRy);

        drawnow;
    end
end

%% -------------------- Plots --------------------
if ~any(ok)
    warning("No valid points. Likely causes: camera not looking along +Y, baseline too small, or ball out of view.");
else
    figure(2); clf;
    plot(Z_true(ok), Z_err(ok), 'o-');
    grid on;
    xlabel('True Depth (m)');
    ylabel('Absolute Error (m)');
    title('Lab 4: Depth Error vs True Depth');

    figure(3); clf;
    plot(Z_true(ok), Z_true(ok), '-'); hold on;
    plot(Z_true(ok), Y_est_all(ok), 'o');
    grid on;
    xlabel('True Depth (m)');
    ylabel('Estimated Depth (m)');
    title('Lab 4: Estimated vs True Depth');
    legend('Ideal (y=x)', 'Measured');
end

%% -------------------- Helper: circle detection --------------------
function [x, y] = detectBallCircle(imgRGB)
    x = NaN; y = NaN;

    gray = rgb2gray(imgRGB);
    gray = im2double(gray);
    gray = adapthisteq(gray);
    gray = imgaussfilt(gray, 1);

    % Adjust radius range as needed
    [centers, ~, metric] = imfindcircles(gray, [10 40], ...
        'ObjectPolarity','bright', ...
        'Sensitivity', 0.98, ...
        'EdgeThreshold', 0.05);

    if isempty(centers)
        return;
    end

    [~, idx] = max(metric);
    x = centers(idx, 1);
    y = centers(idx, 2);
end

function drawCrosshair(ax, x, y)
    if any(isnan([x y])), return; end
    L = 25;
    line(ax, [x-L x+L], [y y], 'LineWidth', 2);
    line(ax, [x x], [y-L y+L], 'LineWidth', 2);
    plot(ax, x, y, 'o', 'MarkerSize', 8, 'LineWidth', 2);
end