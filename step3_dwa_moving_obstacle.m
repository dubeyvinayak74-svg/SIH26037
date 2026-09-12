%% STEP 3: DWA Local Planner with a Moving Obstacle (custom implementation)
clear; clc; close all;

%% 1. Map bounds and start/goal
mapSize = [20 20];
startPose = [2 10 0];
goalPose  = [18 10 0];

%% 2. Moving obstacle setup
obstacleStart = [10 6];
obstacleVelocity = [0 0.6];
obstaclePos = obstacleStart;

%% 3. DWA tuning parameters (all in one struct, easy to adjust later)
params.dt = 0.1;
params.predictTime = 1.5;      % seconds to simulate ahead per candidate
params.minLinVel = 0;
params.maxLinVel = 1.5;
params.maxLinAccel = 1.0;      % m/s^2
params.maxAngVel = 1.5;        % rad/s
params.maxAngAccel = 3.0;      % rad/s^2
params.linVelStep = 0.1;       % resolution of candidate sampling
params.angVelStep = 0.1;
params.obstacleRadius = 0.6;
params.safetyMargin = 0.4;
params.maxClearance = 3;       % caps how much "credit" distance gives beyond this
params.headingWeight = 1.0;
params.clearanceWeight = 1.2;
params.velocityWeight = 0.5;

%% 4. Initialize vehicle
vehiclePose = startPose;
currentLinVel = 0;
currentAngVel = 0;
poseHistory = vehiclePose;
maxSteps = 500;
goalRadius = 0.5;

%% 5. Live plot setup
figure; hold on; axis equal;
xlim([0 mapSize(1)]); ylim([0 mapSize(2)]);
plot(goalPose(1), goalPose(2), 'g*', 'MarkerSize', 12);
carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor','r');
obsMarker = plot(obstaclePos(1), obstaclePos(2), 'ks', 'MarkerFaceColor','k','MarkerSize',10);
trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
title('Step 3: Custom DWA — Reactive Avoidance of Moving Obstacle');
legend('Goal','Vehicle','Moving Obstacle','Trail');

%% 6. Simulation loop
for step = 1:maxSteps
    obstaclePos = obstaclePos + obstacleVelocity * params.dt;

    [currentLinVel, currentAngVel] = dwaControl(vehiclePose, currentLinVel, ...
        currentAngVel, goalPose(1:2), obstaclePos, params);

    vehiclePose(1) = vehiclePose(1) + currentLinVel*cos(vehiclePose(3))*params.dt;
    vehiclePose(2) = vehiclePose(2) + currentLinVel*sin(vehiclePose(3))*params.dt;
    vehiclePose(3) = vehiclePose(3) + currentAngVel*params.dt;
    poseHistory = [poseHistory; vehiclePose];

    set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
    set(obsMarker, 'XData', obstaclePos(1), 'YData', obstaclePos(2));
    set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
    drawnow;

    distToGoal = norm(vehiclePose(1:2) - goalPose(1:2));
    if distToGoal < goalRadius
        fprintf('Goal reached at step %d!\n', step);
        break;
    end
end

if step == maxSteps
    disp('Warning: max steps reached — check tuning.');
end