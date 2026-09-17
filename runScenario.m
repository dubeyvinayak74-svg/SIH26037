function results = runScenario(scenarioName, mapSize, staticObstacleBoxes, ...
    startPose, goalPose, obstacleStarts, obstacleVelocities, dwaParams, maxSteps)
% RUNSCENARIO - Runs the Hybrid A* + DWA pipeline on a given scenario
% setup, and returns collected performance metrics.
%
% Inputs:
%   scenarioName         - string, just for labeling plots/output
%   mapSize               - [width height] in meters
%   staticObstacleBoxes   - cell array of Nx2 coordinate matrices, one per obstacle
%   startPose, goalPose   - [x y theta]
%   obstacleStarts        - Mx2 matrix, one row [x y] per moving obstacle
%   obstacleVelocities    - Mx2 matrix, one row [vx vy] per moving obstacle
%   dwaParams             - struct of DWA tuning parameters
%   maxSteps              - safety limit on simulation length
%
% Output: results struct containing all collected metrics

    resolution = 2;
    map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);
    for i = 1:numel(staticObstacleBoxes)
        setOccupancy(map, staticObstacleBoxes{i}, 1);
    end
    inflate(map, 0.5);

    % --- Hybrid A* global path ---
    stateSpace = stateSpaceSE2([0 mapSize(1); 0 mapSize(2); -pi pi]);
    validator = validatorOccupancyMap(stateSpace);
    validator.Map = map;
    validator.ValidationDistance = 0.1;
    planner = plannerHybridAStar(validator, 'MinTurningRadius', 3, 'MotionPrimitiveLength', 1);

    planTimer = tic;
    pathObj = plan(planner, startPose, goalPose);
    globalPlanTime = toc(planTimer);
    globalPath = pathObj.States;

    % --- Non-inflated obstacle points for DWA ---
    rawMap = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);
    for i = 1:numel(staticObstacleBoxes)
        setOccupancy(rawMap, staticObstacleBoxes{i}, 1);
    end
    [row, col] = find(occupancyMatrix(rawMap) == 1);
    staticObstaclePoints = grid2world(rawMap, [row col]);

    % --- Initialize vehicle and obstacles ---
    vehiclePose = startPose;
    currentLinVel = 0;
    currentAngVel = 0;
    numObstacles = size(obstacleStarts, 1);
    obstaclePositions = obstacleStarts;
    poseHistory = vehiclePose;
    lookaheadIndex = 1;
    goalRadius = 0.6;

    % --- Metrics tracking ---
    dwaCallTimes = [];
    headingChanges = [];
    collisionOccurred = false;
    goalReached = false;
    stepsToComplete = 0;

    % --- Plot setup ---
    figure; show(map); hold on; axis equal;
    plot(globalPath(:,1), globalPath(:,2), 'g--', 'LineWidth', 1);
    carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor','r');
    obsMarkers = gobjects(numObstacles,1);
    for i = 1:numObstacles
        obsMarkers(i) = plot(obstaclePositions(i,1), obstaclePositions(i,2), ...
            'ms', 'MarkerFaceColor','m', 'MarkerSize',10);
    end
    trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
    title(['Scenario: ' scenarioName]);

    for step = 1:maxSteps
        obstaclePositions = obstaclePositions + obstacleVelocities * dwaParams.dt;

        distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
        while distToLookahead < 1.5 && lookaheadIndex < size(globalPath,1)
            lookaheadIndex = lookaheadIndex + 1;
            distToLookahead = norm(vehiclePose(1:2) - globalPath(lookaheadIndex,1:2));
        end
        pathTarget = globalPath(lookaheadIndex, 1:2);

        allObstacles = [staticObstaclePoints; obstaclePositions];

        dwaTimer = tic;
        [currentLinVel, newAngVel] = dwaControl(vehiclePose, currentLinVel, currentAngVel, ...
            pathTarget, allObstacles, dwaParams);
        dwaCallTimes(end+1) = toc(dwaTimer);

        headingChanges(end+1) = abs(newAngVel * dwaParams.dt);
        currentAngVel = newAngVel;

        vehiclePose(1) = vehiclePose(1) + currentLinVel*cos(vehiclePose(3))*dwaParams.dt;
        vehiclePose(2) = vehiclePose(2) + currentLinVel*sin(vehiclePose(3))*dwaParams.dt;
        vehiclePose(3) = vehiclePose(3) + currentAngVel*dwaParams.dt;
        poseHistory = [poseHistory; vehiclePose];

        % --- Collision check against static + ALL moving obstacles ---
        distToStatic = min(sqrt(sum((staticObstaclePoints - vehiclePose(1:2)).^2, 2)));
        distToMoving = min(sqrt(sum((obstaclePositions - vehiclePose(1:2)).^2, 2)));
        if distToStatic < 0.3 || distToMoving < 0.3
            collisionOccurred = true;
        end

        set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
        for i = 1:numObstacles
            set(obsMarkers(i), 'XData', obstaclePositions(i,1), 'YData', obstaclePositions(i,2));
        end
        set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
        drawnow;
        pause(0.02);

        if norm(vehiclePose(1:2) - goalPose(1:2)) < goalRadius
            goalReached = true;
            stepsToComplete = step;
            break;
        end
    end

    if ~goalReached
        stepsToComplete = maxSteps;
    end




    % --- Package results ---
    results.scenarioName = scenarioName;
    results.goalReached = goalReached;
    results.collisionOccurred = collisionOccurred;
    results.stepsToComplete = stepsToComplete;
    results.completionTimeSeconds = stepsToComplete * dwaParams.dt;
    results.globalPlanTimeSeconds = globalPlanTime;
    results.avgReplanLatencyMs = mean(dwaCallTimes) * 1000;
    results.maxReplanLatencyMs = max(dwaCallTimes) * 1000;
    results.avgHeadingChangePerStep = mean(headingChanges);
    results.poseHistory = poseHistory;
    
end
