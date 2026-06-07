%% main_vision_sorting.m
% Vision-guided sorting controller for the 5-DOF robotic arm.
%
% Extracted and cleaned from Appendix A of the graduation project report.
% The script performs:
%   1) Image acquisition from a USB camera.
%   2) Undistortion using MATLAB camera calibration parameters.
%   3) Color/shape detection and centroid/orientation extraction.
%   4) Pixel/world scaling and object-to-base coordinate transformation.
%   5) Analytical inverse kinematics.
%   6) I2C transmission of desired joint angles to Arduino Mega.
%
% IMPORTANT:
% - Update serialPort, i2cAddress, calibrationFile, and cameraName for your PC.
% - Verify all placeAng values using your real bin locations before running.
% - Run first with motors disabled or robot lifted for safety.

clc;
clear;
close all;

%% User configuration
serialPort      = 'COM7';                    % Example on Windows: 'COM7'
i2cAddress      = '0x08';                    % Arduino Mega I2C address
calibrationFile = 'F:\AAAAA\calibration\matlab.mat';
cameraName      = 'Sirius USB2.0 Camera';

%% Robot geometric parameters [mm]
a1 = 26.75;
d1 = 75;
a2 = 156.5;
a3 = 130;
a4 = 25.25;
d5 = 135.25;      % Original note in PDF: d5 = 114.25
L  = 20;

%% Pixel-to-world scale factors [cm/pixel]
scale1 = 291.5/500;
scale2 = 220/380;

%% Eye-to-hand transform setup
% Origin of world frame relative to base frame [mm or consistent project units]
P0_w = [145 -138 22];

% Object position relative to world frame
Pw_obj = [0 0 -20];

% World frame relative to base frame
T0_w = [0, 1,  0, P0_w(1);
        1, 0,  0, P0_w(2);
        0, 0, -1, P0_w(3);
        0, 0,  0, 1];

%% State flags
EN_witingPos  = 1;   % Original variable name preserved
EN_writing    = 1;
EN_pickupPos  = 0;
EN_placePos   = 0;
latchPlacepos = 1;
objSelect     = 0;
objType       = 0;

%% Gripper and command vectors
Gopen  = 160;
Gclose = [125 125 125];

initialAng = [0 0 0 0 0 Gopen 1 1]; %#ok<NASGU> % Initial position
witingAng  = [45 0 0 0 0 Gopen 1 0];          % Waiting position
pickupAng  = [0 0 0 0 0 Gopen 1 0];           % Pickup command
placeAng   = [0 0 0 0 0 Gopen 1 0];           % Default place command

%% Hardware initialization
arduinoObj = arduino(serialPort);
slave      = i2cdev(arduinoObj, i2cAddress);

load(calibrationFile, 'cameraParams');
mycam = webcam(cameraName);

% Optional beep signal
fs = 8000;
ts = 1/fs;
n  = 1:10000;
f  = 500;
y  = 5*sin(2*pi*f*ts*n); %#ok<NASGU>

tic;

%% Main loop
while true
    %% Image acquisition and preprocessing
    imOrig = snapshot(mycam);
    [I, ~] = undistortImage(imOrig, cameraParams);

    gr = rgb2gray(I);
    BW = ~imbinarize(gr, 'adaptive', ...
        'ForegroundPolarity', 'dark', ...
        'Sensitivity', 0.5);

    mg = imfill(BW, 'holes');
    mg = bwareaopen(mg, 2000);

    [labeled, num] = bwlabel(mg, 8);
    Bound = bwboundaries(mg);
    info  = regionprops(labeled, 'all');

    if num == 0
        objSelect = 0;
        drawnow;
        continue;
    end

    centroids = cat(1, info.Centroid) .* [scale1 scale2];
    Xw_obj = centroids(:, 1);
    Yw_obj = centroids(:, 2);
    orientationsDeg = cat(1, info.Orientation);

    %% Color segmentation
    redBand   = imsubtract(I(:,:,1), gr);
    greenBand = imsubtract(I(:,:,2), gr);
    blueBand  = imsubtract(I(:,:,3), gr);

    redThreshold   = 50;
    greenThreshold = 40;
    blueThreshold  = 40;

    redMask   = redBand   > redThreshold;
    greenMask = greenBand > greenThreshold;
    blueMask  = blueBand  > blueThreshold;

    R = bwareaopen(imfill(redMask,   'holes'), 2000);
    G = bwareaopen(imfill(greenMask, 'holes'), 2000);
    B = bwareaopen(imfill(blueMask,  'holes'), 2000);

    RobjNum = unique(immultiply(labeled, R)); RobjNum = RobjNum(RobjNum > 0);
    GobjNum = unique(immultiply(labeled, G)); GobjNum = GobjNum(GobjNum > 0);
    BobjNum = unique(immultiply(labeled, B)); BobjNum = BobjNum(BobjNum > 0);

    shapeFactor = [info.Perimeter].^2 ./ (4*pi*[info.Area]);

    %% Display detection result
    RI = imref2d(size(I), scale1, scale2);
    imshow(I, RI);
    grid on;
    hold on;

    objSelect = 0;

    for i = 1:num
        plot(centroids(i,1), centroids(i,2), 'g+', 'LineWidth', 10);

        if ismember(i, RobjNum)
            colorIndex = 1;
        elseif ismember(i, GobjNum)
            colorIndex = 2;
        elseif ismember(i, BobjNum)
            colorIndex = 3;
        else
            colorIndex = 0;
        end

        [currentObjType, objLabel] = findObjType(shapeFactor(i), colorIndex);

        % Select first valid object. Replace this by nearest-to-origin selection if needed.
        if currentObjType > 0 && objSelect == 0
            objSelect = i;
            objType = currentObjType;
        end

        text(centroids(i,1)+20, centroids(i,2), ...
            ['(', mat2str(round(centroids(i,1),2)), ',', ...
                  mat2str(round(centroids(i,2),2)), ')'], ...
            'Color', 'red', 'FontSize', 15);

        text(centroids(i,1)+20*scale1, centroids(i,2)+20*scale2, ...
            ['(', objLabel, ', ', mat2str(round(orientationsDeg(i),2)), ' deg)'], ...
            'Color', 'g', 'FontSize', 15);

        boundary = Bound{i};
        plot(boundary(:,2)*scale1, boundary(:,1)*scale2, 'b', 'LineWidth', 2);
    end

    % Workspace/calibration reference points
    plot(320*scale1, 240*scale2, 'r+', 'LineWidth', 2);
    plot(50*scale1,   50*scale2, 'r+', 'LineWidth', 2);
    plot(590*scale1,  50*scale2, 'r+', 'LineWidth', 2);
    plot(50*scale1,  430*scale2, 'r+', 'LineWidth', 2);
    plot(590*scale1, 430*scale2, 'r+', 'LineWidth', 2);

    drawnow;
    hold off;

    %% Inverse kinematics for selected object
    if objSelect > 0 && objSelect <= length(Xw_obj)
        Pw_obj(1) = Xw_obj(objSelect);
        Pw_obj(2) = Yw_obj(objSelect);
        Pw_obj(3) = -20;

        thetaDeg = orientationsDeg(objSelect);
        thetaRad = deg2rad(thetaDeg); % regionprops Orientation is in degrees

        Tw_obj = [cos(thetaRad), -sin(thetaRad), 0, Pw_obj(1);
                  sin(thetaRad),  cos(thetaRad), 0, Pw_obj(2);
                  0,              0,             1, Pw_obj(3);
                  0,              0,             0, 1];

        % Object pose relative to base frame
        T0_obj = T0_w * Tw_obj;

        x0 = T0_obj(1,4);
        y0 = T0_obj(2,4);
        z0 = T0_obj(3,4);

        q0 = atan2(x0, y0);
        q1 = asin(L/sqrt(x0^2 + y0^2)) - q0 - (pi/2);

        x = sqrt(x0^2 + y0^2 - L^2) * cos(q1);
        y = sqrt(x0^2 + y0^2 - L^2) * sin(q1); %#ok<NASGU>
        z = z0;

        % gamma aligns the gripper z-axis with the base z-axis
        gamma = 0;

        rg = sqrt(x^2 + y^2) - a1;
        zg = z - d1;
        zw = zg + (d5*cos(gamma) - a4*sin(gamma));
        rw = rg - (a4*cos(gamma) + d5*sin(gamma));

        cosq3 = (zw^2 + rw^2 - a2^2 - a3^2)/(2*a2*a3);

        if abs(cosq3) > 1
            beep;
            EN_witingPos  = 1;
            EN_writing    = 1;
            EN_pickupPos  = 0;
            EN_placePos   = 0;
            latchPlacepos = 1;
            objSelect     = 0;
            continue;
        end

        sinq3 = sqrt(1 - cosq3^2);

        q3(1) = atan2( sinq3, cosq3);
        q3(2) = atan2(-sinq3, cosq3);

        q2(1) = atan2(zw, rw) - atan2(a3*sin(q3(1)), a2 + a3*cos(q3(1)));
        q2(2) = atan2(zw, rw) - atan2(a3*sin(q3(2)), a2 + a3*cos(q3(2)));

        q4(1) = gamma - (q2(1) + q3(1));
        q4(2) = gamma - (q2(2) + q3(2));

        q1 = rad2deg(q1);
        q2 = rad2deg(q2);
        q3 = rad2deg(q3);
        q4 = rad2deg(q4);

        Th(1) = q1;
        Th(2) = q2(2) - 90;
        Th(3) = q3(2) + 90;
        Th(4) = q4(2) - 90;

        % Gripper roll angle based on object orientation
        if thetaDeg >= 0
            Th(5) = Th(1) - thetaDeg + 90;
        else
            Th(5) = Th(1) - (90 + thetaDeg);
        end

        TH = round(Th);
        pickupAng = [TH Gopen 1 0];

        if latchPlacepos == 1
            placeAng = CallPlaceAng(objType);
        end
    else
        objSelect = 0;
    end

    %% Handshake with Arduino and command sending
    if toc >= 15
        EN_writing = read(slave, 1, 'int8');
        tic;
    end

    if EN_witingPos == 1 && EN_writing == 1
        write(slave, witingAng, 'int16');
        EN_writing    = 0;
        EN_witingPos  = 0;
        EN_pickupPos  = 1;
        latchPlacepos = 1;

    elseif EN_pickupPos == 1 && EN_writing == 1 && objSelect > 0
        write(slave, pickupAng, 'int16');
        EN_writing    = 0;
        EN_witingPos  = 0;
        EN_pickupPos  = 0;
        EN_placePos   = 1;
        latchPlacepos = 0;
        objSelect     = 0;

    elseif EN_placePos == 1 && EN_writing == 1
        write(slave, placeAng, 'int16');
        EN_writing    = 0;
        EN_pickupPos  = 0;
        EN_witingPos  = 1;
        EN_placePos   = 0;
        latchPlacepos = 1;
    end
end
