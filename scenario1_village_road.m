%% SCENARIO 1: Unmarked Village Road
clear; clc; close all;

%% Map dimensions
mapSize = [24 12];

%% Static obstacles
staticObstacleBoxes = {
    [6 3; 6 4; 7 3; 7 4];
    [14 8; 14 9; 15 8; 15 9]
    };

%% Start and goal
startPose = [1 6 0];
goalPose  = [22 6 0];

%% Moving obstacle (single obstacle, now passed as 1-row matrices)
obstacleStarts = [11 4.3];
obstacleVelocities = [0 0.35];

%% DWA parameters
dwaParams.dt = 0.1;
dwaParams.predictTime = 1.5;
dwaParams.minLinVel = 0;
dwaParams.maxLinVel = 1.5;
dwaParams.maxLinAccel = 1.0;
dwaParams.maxAngVel = 1.5;
dwaParams.maxAngAccel = 3.0;
dwaParams.linVelStep = 0.1;
dwaParams.angVelStep = 0.1;
dwaParams.obstacleRadius = 0.6;
dwaParams.safetyMargin = 0.4;
dwaParams.maxClearance = 3;
dwaParams.headingWeight = 0.6;
dwaParams.clearanceWeight = 1.0;
dwaParams.velocityWeight = 0.8;
dwaParams.progressWeight = 2.0;
dwaParams.debug = false;

maxSteps = 600;

%% Run scenario
results = runScenario('Unmarked Village Road', mapSize, staticObstacleBoxes, ...
    startPose, goalPose, obstacleStarts, obstacleVelocities, dwaParams, maxSteps);

%% Debug: confirm real heading-change value
fprintf('DEBUG raw heading value: %.10f\n', results.avgHeadingChangePerStep);

%% Display results summary
fprintf('\n=== Scenario 1 Results: Unmarked Village Road ===\n');
fprintf('Goal Reached:            %d\n', results.goalReached);
fprintf('Collision Occurred:      %d\n', results.collisionOccurred);
fprintf('Completion Time:         %.2f s\n', results.completionTimeSeconds);
fprintf('Global Plan Time:        %.4f s\n', results.globalPlanTimeSeconds);
fprintf('Avg Replan Latency:      %.3f ms\n', results.avgReplanLatencyMs);
fprintf('Max Replan Latency:      %.3f ms\n', results.maxReplanLatencyMs);
fprintf('Avg Heading Change/Step: %.6f rad\n', results.avgHeadingChangePerStep);