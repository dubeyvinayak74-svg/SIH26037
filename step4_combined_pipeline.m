%% STEP 4: Combined Hybrid A* (global) + DWA (local) Pipeline
clear; clc; close all;

%% 1. Build map with STATIC obstacles (same as Step 1)
mapSize = [20 20];
resolution = 2;
map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);

obstacle1 = [8 8; 8 9; 9 8; 9 9];
obstacle2 = [12 4; 12 5; 13 4; 13 5];
setOccupancy(map, obstacle1, 1);
setOccupancy(map, obstacle2, 1);
inflate(map, 0.5);

%% 2. Plan GLOBAL path with Hybrid A*
stateSpace = stateSpaceSE2([0 mapSize(1); 0 mapSize(2); -pi pi]);
validator = validatorOccupancyMap(stateSpace);
validator.Map = map;
validator.ValidationDistance = 0.1;

planner = plannerHybridAStar(validator, 'MinTurningRadius', 3, 'MotionPrimitiveLength', 1);

startPose = [2 2 0];
goalPose  = [18 18 pi/2];
pathObj = plan(planner, startPose, goalPose);
globalPath = pathObj.States;   % Nx3: [x y theta]

%% 3. Convert static obstacles into a point list
% Uses a SEPARATE, non-inflated map here. The main "map" variable above
% is inflated by 0.5m for Hybrid A*'s global planning. dwaControl.m
% already adds its own buffer (obstacleRadius + safetyMargin) around
% every obstacle point, so using the inflated map here would stack two
% safety margins on top of each other.
rawMap = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);
setOccupancy(rawMap, obstacle1, 1);
setOccupancy(rawMap, obstacle2, 1);

[row, col] = find(occupancyMatrix(rawMap) == 1);
staticObstaclePoints = grid2world(rawMap, [row col]);

%% 4. Moving obstacle setup
obstacleStart = [14 7];
obstacleVelocity = [-0.5 0];
obstaclePos = obstacleStart;

%% 5. DWA parameters (REBALANCED — see explanation below)
params.dt = 0.1;
params.predictTime = 1.5;
params.minLinVel = 0;
params.maxLinVel = 1.5;
params.maxLinAccel = 1.0;
params.maxAngVel = 1.5;
params.maxAngAccel = 3.0;
params.linVelStep = 0.1;
params.angVelStep = 0.1;
params.obstacleRadius = 0.6;
params.safetyMargin = 0.4;
params.maxClearance = 3;
params.headingWeight = 0.6;      % reduced from 1.0
params.clearanceWeight = 1.0;    % reduced from 1.2
params.velocityWeight = 0.8;     % increased from 0.5
params.progressWeight = 2.0;     % NEW — rewards actually closing distance to goal
params.debug = false;            % set true if we need diagnostics again

%% 6. Initialize vehicle
vehiclePose = startPose;
currentLinVel = 0;
currentAngVel = 0;
poseHistory = vehiclePose;
maxSteps = 800;
goalRadius = 0.5;
lookaheadIndex = 1;

%% 7. Plot setup
figure; show(map); hold on; axis equal;
pathLine = plot(globalPath(:,1), globalPath(:,2), 'g--', 'LineWidth', 1);
plot(staticObstaclePoints(:,1), staticObstaclePoints(:,2), 'y.', 'MarkerSize', 4);
carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor','r');
obsMarker = plot(obstaclePos(1), obstaclePos(2), 'ms', 'MarkerFaceColor','m', 'MarkerSize',10);
trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
title('Step 4: Hybrid A* (global) + DWA (local) Combined');

legend([pathLine, carMarker, obsMarker, trailLine], ...
    {'Global Path','Vehicle','Moving Obstacle','Trail'});

%% 8. Simulation loop
for step = 1:maxSteps
    obstaclePos = obstaclePos + obstacleVelocity * params.dt;

    % --- Advance lookahead point along the global path ---
    distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
    while distToLookahead < 1.5 && lookaheadIndex < size(globalPath,1)
        lookaheadIndex = lookaheadIndex + 1;
        distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
    end
    currentTarget = globalPath(lookaheadIndex, 1:2);

    % --- Combine static + moving obstacles ---
    allObstacles = [staticObstaclePoints; obstaclePos];

    % --- Run DWA ---
    [currentLinVel, currentAngVel] = dwaControl(vehiclePose, currentLinVel, ...
        currentAngVel, currentTarget, allObstacles, params);

    % --- Update vehicle position (bicycle model) ---
    vehiclePose(1) = vehiclePose(1) + currentLinVel*cos(vehiclePose(3))*params.dt;
    vehiclePose(2) = vehiclePose(2) + currentLinVel*sin(vehiclePose(3))*params.dt;
    vehiclePose(3) = vehiclePose(3) + currentAngVel*params.dt;
    poseHistory = [poseHistory; vehiclePose];

    % --- Update plot ---
    set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
    set(obsMarker, 'XData', obstaclePos(1), 'YData', obstaclePos(2));
    set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
    drawnow;
    pause(0.03);

    % --- Diagnostic printout every 20 steps ---
    if mod(step, 20) == 0
        fprintf('Step %d | pose=(%.2f, %.2f) | vel=%.2f | targetIdx=%d/%d\n', ...
            step, vehiclePose(1), vehiclePose(2), currentLinVel, lookaheadIndex, size(globalPath,1));
    end

    % --- Stop the obstacle once it's off the relevant area ---
    if obstaclePos(1) > mapSize(1) || obstaclePos(2) > mapSize(2)
        obstacleVelocity = [0 0];
    end

    % --- Check final goal reached ---
    distToGoal = norm(vehiclePose(1:2) - goalPose(1:2));
    if distToGoal < goalRadius
        fprintf('Goal reached at step %d!\n', step);
        break;
    end
end

if step == maxSteps
    disp('Warning: max steps reached — check tuning.');
end