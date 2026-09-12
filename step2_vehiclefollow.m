%% STEP 2: Vehicle Model + Path Following (Pure Pursuit)
% Builds on Step 1's path. Simulates a car physically driving along it
% using a kinematic bicycle model, steered by a Pure Pursuit controller.

clear; clc; close all;

%% 1. Rebuild the map and path from Step 1
% (Re-running Step 1's setup here so this script works standalone)
mapSize = [20 20];
resolution = 2;
map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);

obstacle1 = [8 8; 8 9; 9 8; 9 9];
obstacle2 = [12 4; 12 5; 13 4; 13 5];
setOccupancy(map, obstacle1, 1);
setOccupancy(map, obstacle2, 1);
inflate(map, 0.5);

stateSpace = stateSpaceSE2([0 mapSize(1); 0 mapSize(2); -pi pi]);
validator = validatorOccupancyMap(stateSpace);
validator.Map = map;
validator.ValidationDistance = 0.1;

planner = plannerHybridAStar(validator, ...
    'MinTurningRadius', 3, ...
    'MotionPrimitiveLength', 1);

startPose = [2 2 0];
goalPose  = [18 18 pi/2];

pathObj = plan(planner, startPose, goalPose);
pathPoints = pathObj.States;   % Nx3 matrix: [x y theta] for each path point

%% 2. Set up the Pure Pursuit controller
% This MATLAB object handles the "look ahead and steer toward it" logic for us
controller = controllerPurePursuit;
controller.Waypoints = pathPoints(:,1:2);   % only needs x,y, not theta
controller.LookaheadDistance = 2;           % meters - how far ahead it looks
controller.DesiredLinearVelocity = 1;       % m/s - constant speed for now
controller.MaxAngularVelocity = 2;          % rad/s - steering limit

%% 3. Initialize vehicle state
% [x, y, theta] - starting exactly at the planned start pose
vehiclePose = startPose;
dt = 0.1;              % simulation time step (seconds)
goalRadius = 0.5;       % how close counts as "reached the goal"
maxSteps = 500;         % safety limit so it can't loop forever

% Store history for plotting the traveled trail
poseHistory = vehiclePose;

%% 4. Set up live plot
figure;
show(map); hold on;
plot(pathPoints(:,1), pathPoints(:,2), 'g--', 'LineWidth', 1.5); % planned path
carMarker = plot(vehiclePose(1), vehiclePose(2), 'ro', 'MarkerFaceColor', 'r');
trailLine = plot(vehiclePose(1), vehiclePose(2), 'b-', 'LineWidth', 1.5);
title('Step 2: Vehicle Following Planned Path (Pure Pursuit)');
legend('Obstacles','Planned Path','Vehicle','Traveled Trail');

%% 5. Simulation loop
for step = 1:maxSteps

    % Ask the controller: "given where I am now, how should I steer?"
    [linVel, angVel] = controller(vehiclePose);

    % --- Kinematic bicycle model update ---
    % This is the core physics: how position/heading change given
    % current speed and turning rate, over one small time step (dt)
    x = vehiclePose(1) + linVel * cos(vehiclePose(3)) * dt;
    y = vehiclePose(2) + linVel * sin(vehiclePose(3)) * dt;
    theta = vehiclePose(3) + angVel * dt;

    vehiclePose = [x, y, theta];
    poseHistory = [poseHistory; vehiclePose];

    % Update the plot to show movement
    set(carMarker, 'XData', vehiclePose(1), 'YData', vehiclePose(2));
    set(trailLine, 'XData', poseHistory(:,1), 'YData', poseHistory(:,2));
    drawnow;

    % Check if we've reached the goal
    distToGoal = sqrt((vehiclePose(1)-goalPose(1))^2 + (vehiclePose(2)-goalPose(2))^2);
    if distToGoal < goalRadius
        disp('Goal reached!');
        break;
    end
end

if step == maxSteps
    disp('Warning: max steps reached before goal — check tuning.');
end