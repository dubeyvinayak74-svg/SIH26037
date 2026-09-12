 %% STEP 6: Full Pipeline — Hybrid A* (global) + DWA (safety veto) + MPC (smooth control)
clear; clc; close all;

%% 1. Map + static obstacles
mapSize = [20 20];
resolution = 2;
map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);
obstacle1 = [8 8; 8 9; 9 8; 9 9];
obstacle2 = [12 4; 12 5; 13 4; 13 5];
setOccupancy(map, obstacle1, 1);
setOccupancy(map, obstacle2, 1);
inflate(map, 0.5);

%% 2. Hybrid A* global path
stateSpace = stateSpaceSE2([0 mapSize(1); 0 mapSize(2); -pi pi]);
validator = validatorOccupancyMap(stateSpace);
validator.Map = map;
validator.ValidationDistance = 0.1;
planner = plannerHybridAStar(validator, 'MinTurningRadius', 3, 'MotionPrimitiveLength', 1);
startPose = [2 2 0];
goalPose  = [18 18 pi/2];
pathObj = plan(planner, startPose, goalPose);
globalPath = pathObj.States;

%% 3. Static obstacle points for DWA (non-inflated)
rawMap = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);
setOccupancy(rawMap, obstacle1, 1);
setOccupancy(rawMap, obstacle2, 1);
[row, col] = find(occupancyMatrix(rawMap) == 1);
staticObstaclePoints = grid2world(rawMap, [row col]);

%% 4. Moving obstacle
obstacleStart = [14 7];
obstacleVelocity = [-0.7 0];
obstaclePos = obstacleStart;

%% 5. DWA parameters
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

%% 6. MPC setup
wheelbase = 2.5;
dt = 0.1;
theta0 = 0;
A = [0 0 -sin(theta0); 0 0 cos(theta0); 0 0 0];
B = [cos(theta0) 0; sin(theta0) 0; 0 1/wheelbase];
C = eye(3); D = zeros(3,2);
plant = c2d(ss(A,B,C,D), dt);

mpcController = mpc(plant, dt);
mpcController.PredictionHorizon = 15;
mpcController.ControlHorizon = 3;
mpcController.MV(1).Min = 0; mpcController.MV(1).Max = 1.5;
mpcController.MV(2).Min = -0.6; mpcController.MV(2).Max = 0.6;
mpcController.MV(1).RateMin = -0.3; mpcController.MV(1).RateMax = 0.3;
mpcController.MV(2).RateMin = -0.2; mpcController.MV(2).RateMax = 0.2;
mpcController.Weights.OutputVariables = [1 1 0.5];
mpcController.Weights.ManipulatedVariables = [0.1 0.1];
mpcController.Weights.ManipulatedVariablesRate = [0.3 0.3];
mpcState = mpcstate(mpcController);

%% 7. Initialize vehicle
vehiclePose = startPose;
currentLinVel = 0;
currentAngVel = 0;
lastSteer = 0;
poseHistory = vehiclePose;
maxSteps = 200;
goalRadius = 0.6;
lookaheadIndex = 1;

%% 8. Plot setup
figure; show(map); hold on; axis equal;
pathLine = plot(globalPath(:,1), globalPath(:,2), 'g--', 'LineWidth', 1);
plot(staticObstaclePoints(:,1), staticObstaclePoints(:,2), 'y.', 'MarkerSize', 4);
carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor','r');
obsMarker = plot(obstaclePos(1), obstaclePos(2), 'ms', 'MarkerFaceColor','m', 'MarkerSize',10);
trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
title('Step 6: Full Pipeline — Hybrid A* + DWA (safety veto) + MPC');
legend([pathLine, carMarker, obsMarker, trailLine], {'Global Path','Vehicle','Moving Obstacle','Trail'});

%% 9. Simulation loop
for step = 1:maxSteps
    obstaclePos = obstaclePos + obstacleVelocity * dwaParams.dt;

    distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
    while distToLookahead < 1.5 && lookaheadIndex < size(globalPath,1)
        lookaheadIndex = lookaheadIndex + 1;
        distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
    end
    pathTarget = globalPath(lookaheadIndex, 1:2);

    allObstacles = [staticObstaclePoints; obstaclePos];
    [dwaLinVel, dwaAngVel] = dwaControl(vehiclePose, currentLinVel, currentAngVel, ...
        pathTarget, allObstacles, dwaParams);

    distToNearestObs = min(sqrt(sum((allObstacles - vehiclePose(1:2)).^2, 2)));
    avoidanceNeeded = distToNearestObs < 2.5;

    if avoidanceNeeded
        projTime = 0.5;
        mpcTargetX = vehiclePose(1) + dwaLinVel*cos(vehiclePose(3))*projTime;
        mpcTargetY = vehiclePose(2) + dwaLinVel*sin(vehiclePose(3))*projTime;
        mpcTargetTheta = vehiclePose(3) + dwaAngVel*projTime;
        mpcTarget = [mpcTargetX, mpcTargetY, mpcTargetTheta];
    else
        rawHeadingToTarget = atan2(pathTarget(2)-vehiclePose(2), pathTarget(1)-vehiclePose(1));
        headingError = wrapToPi(rawHeadingToTarget - vehiclePose(3));
        maxHeadingStep = 0.3;
        cappedHeadingError = max(min(headingError, maxHeadingStep), -maxHeadingStep);
        headingToTarget = vehiclePose(3) + cappedHeadingError;
        mpcTarget = [pathTarget(1), pathTarget(2), headingToTarget];
    end

    currentTheta = vehiclePose(3);
    A = [0 0 -sin(currentTheta); 
        0 0  cos(currentTheta); 
        0 0  0];
    B = [cos(currentTheta) 0; 
        sin(currentTheta) 0; 
        0                 1/wheelbase];
    updatedPlant = c2d(ss(A,B,C,D), dt);

    Nominal = struct('X', vehiclePose', 'U', [currentLinVel; lastSteer], ...
                      'Y', vehiclePose', 'DX', zeros(3,1));

    [mv, Info] = mpcmoveAdaptive(mpcController, mpcState, updatedPlant, Nominal, ...
        vehiclePose', mpcTarget');

    % --- DIAGNOSTIC: correct field names per MATLAB docs ---
    if step >= 95 && step <= 130
        fprintf('  [MPC] step=%d | mv=[%.2f,%.2f] | Iterations=%d | QPCode=%s | target=(%.2f,%.2f,%.2f) | curTheta=%.2f\n', ...
            step, mv(1), mv(2), Info.Iterations, Info.QPCode, mpcTarget(1), mpcTarget(2), mpcTarget(3), vehiclePose(3));
    end

    currentLinVel = mv(1);
    lastSteer = mv(2);
    currentAngVel = mv(2) * currentLinVel / wheelbase;

    vehiclePose(1) = vehiclePose(1) + currentLinVel*cos(vehiclePose(3))*dt;
    vehiclePose(2) = vehiclePose(2) + currentLinVel*sin(vehiclePose(3))*dt;
    vehiclePose(3) = vehiclePose(3) + (currentLinVel/wheelbase)*tan(mv(2))*dt;
    poseHistory = [poseHistory; vehiclePose];

    set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
    set(obsMarker, 'XData', obstaclePos(1), 'YData', obstaclePos(2));
    set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
    drawnow;
    pause(0.02);

    if mod(step,20)==0
        fprintf('Step %d | pose=(%.2f,%.2f) | vel=%.2f | lookaheadIdx=%d/%d | target=(%.2f,%.2f) | avoid=%d\n', ...
            step, vehiclePose(1), vehiclePose(2), currentLinVel, lookaheadIndex, size(globalPath,1), ...
            mpcTarget(1), mpcTarget(2), avoidanceNeeded);
    end

    if obstaclePos(1) > mapSize(1) || obstaclePos(2) > mapSize(2)
        obstacleVelocity = [0 0];
    end

    if norm(vehiclePose(1:2) - goalPose(1:2)) < goalRadius
        fprintf('Goal reached at step %d!\n', step);
        break;
    end
end

if step == maxSteps
    disp('Warning: max steps reached — check tuning.');
end