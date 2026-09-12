%% STEP 1: Static Hybrid A* Path Planner
% This script builds a simple map with obstacles, and asks Hybrid A*
% to find a drivable path from a start point to a goal point.

clear; clc; close all;

%% 1. Create the map (our simplified "village road")
% binaryOccupancyMap: 1 = obstacle (blocked), 0 = free space
% We're making a 20m x 20m area, with a resolution of 2 cells per meter
mapSize = [20 20];       % width x height in meters
resolution = 2;          % cells per meter (higher = more detail, slower)

map = binaryOccupancyMap(mapSize(1), mapSize(2), resolution);

% Add a couple of "parked cart" obstacles manually.
% setOccupancy takes [x y] coordinates and sets them to 1 (blocked).
obstacle1 = [8 8; 8 9; 9 8; 9 9];   % a small 1x1m block near the middle
obstacle2 = [12 4; 12 5; 13 4; 13 5]; % another block, offset

setOccupancy(map, obstacle1, 1);
setOccupancy(map, obstacle2, 1);

% Inflate obstacles slightly so the planner keeps a safety margin
% (this simulates "don't graze the cart", not just "don't hit its center")
inflate(map, 0.5);   % inflate by 0.5 meters in all directions

%% 2. Define the state space (x, y, heading angle)
% SE2 = "Special Euclidean group in 2D" — just means (x, y, theta)
% We tell it the bounds of x, y (matching our map) and theta (-pi to pi)
stateSpace = stateSpaceSE2([0 mapSize(1); 0 mapSize(2); -pi pi]);

%% 3. Define the validator (checks if a state is free)
validator = validatorOccupancyMap(stateSpace);
validator.Map = map;
validator.ValidationDistance = 0.1;  % how finely it checks along a path

%% 4. Create the Hybrid A* planner
planner = plannerHybridAStar(validator, ...
    'MinTurningRadius', 3, ...   % smallest circle radius the car can turn (meters)
    'MotionPrimitiveLength', 1); % length of each small path segment considered

%% 5. Define start and goal poses [x y theta]
% theta = 0 means "facing along positive x-axis"
startPose = [2 2 0];      % start near bottom-left, facing right
goalPose  = [18 18 pi/2]; % goal near top-right, facing "up"

%% 6. Plan the path
pathObj = plan(planner, startPose, goalPose);

%% 7. Visualize the result
show(planner)
title('Hybrid A* Path — Static Village Road Obstacles')