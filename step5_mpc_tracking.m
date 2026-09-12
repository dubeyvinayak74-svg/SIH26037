%% STEP 5: MPC Controller for Smooth Path Tracking
% Replaces the raw bicycle-model update with an MPC controller that
% computes smooth acceleration/steering commands, respecting the
% vehicle's real limits, instead of DWA's simpler reactive choice.

clear; clc; close all;

%% 1. Vehicle parameters
dt = 0.1;
wheelbase = 2.5;   % meters — distance between front and rear axle (typical car)

%% 2. Build a LINEARIZED bicycle model around a reference heading
% States: [x, y, theta]   Inputs: [velocity, steering angle]
theta0 = 0;

A = [0 0 -sin(theta0); 
     0 0  cos(theta0); 
     0 0  0];
B = [cos(theta0) 0; 
     sin(theta0) 0; 
     0            1/wheelbase];
C = eye(3);
D = zeros(3,2);

plant = ss(A, B, C, D);
plant = c2d(plant, dt);   % convert to discrete time, matching our dt

%% 3. Create the MPC controller
mpcController = mpc(plant, dt);
mpcController.PredictionHorizon = 15;
mpcController.ControlHorizon = 3;

% --- Constrain inputs to realistic vehicle limits ---
mpcController.MV(1).Min = 0;
mpcController.MV(1).Max = 1.5;
mpcController.MV(2).Min = -0.6;
mpcController.MV(2).Max = 0.6;

% --- Constrain how FAST inputs can change (this enforces smoothness) ---
mpcController.MV(1).RateMin = -0.3;
mpcController.MV(1).RateMax = 0.3;
mpcController.MV(2).RateMin = -0.2;
mpcController.MV(2).RateMax = 0.2;

% Add these lines right after creating mpcController (Section 3):
mpcController.Weights.OutputVariables = [1 1 0.5];   % weight x, y AND theta now
mpcController.Weights.ManipulatedVariables = [0.1 0.1];   % small penalty on using large inputs
mpcController.Weights.ManipulatedVariablesRate = [0.3 0.3];   % stronger smoothness penalty

%% 4. Simulate: track a simple reference path (straight line for this test)
refX = 0:0.2:15;
refY = zeros(size(refX));
refPath = [refX', refY'];

vehiclePose = [0 0.5 0];   % start slightly OFF the line, so we can SEE it correct itself
poseHistory = vehiclePose;
maxSteps = 300;
goalRadius = 0.6;

mpcState = mpcstate(mpcController);   % holds MPC's internal state between calls

figure; hold on; axis equal;
plot(refPath(:,1), refPath(:,2), 'g--', 'LineWidth', 1);
carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor','r');
trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
title('Step 5: MPC Path Tracking (Straight Line Test)');
legend('Reference Path','Vehicle','Trail');

%% 5. Simulation loop
targetIndex = 1;
for step = 1:maxSteps

    % Pick the current target point (simple lookahead, like Step 4)
    distToTarget = norm(vehiclePose(1:2) - refPath(targetIndex,:));
    while distToTarget < 1.0 && targetIndex < size(refPath,1)
        targetIndex = targetIndex + 1;
        distToTarget = norm(vehiclePose(1:2) - refPath(targetIndex,:));
    end
    target = [refPath(targetIndex,:), 0];   % target state: [x y theta=0]

    % --- Run MPC: given current state, compute best next input ---
    [mv, Info] = mpcmove(mpcController, mpcState, vehiclePose', target');
    velCmd = mv(1);
    steerCmd = mv(2);

    % --- Apply to REAL nonlinear bicycle model (not the linear approx) ---
    vehiclePose(1) = vehiclePose(1) + velCmd*cos(vehiclePose(3))*dt;
    vehiclePose(2) = vehiclePose(2) + velCmd*sin(vehiclePose(3))*dt;
    vehiclePose(3) = vehiclePose(3) + (velCmd/wheelbase)*tan(steerCmd)*dt;
    poseHistory = [poseHistory; vehiclePose];

    set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
    set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
    drawnow;
    pause(0.02);

    if mod(step,20)==0
        fprintf('Step %d | pose=(%.2f,%.2f,%.2f) | vel=%.2f | steer=%.2f\n', ...
            step, vehiclePose(1), vehiclePose(2), vehiclePose(3), velCmd, steerCmd);
    end

    if norm(vehiclePose(1:2) - refPath(end,:)) < goalRadius
        fprintf('Reached end of path at step %d\n', step);
        break;
    end
end