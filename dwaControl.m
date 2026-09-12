function [bestLinVel, bestAngVel] = dwaControl(pose, currentLinVel, currentAngVel, goal, obstacles, params)
% DWACONTROL - Dynamic Window Approach for local obstacle avoidance

dt = params.dt;
predictTime = params.predictTime;

minLin = max(params.minLinVel, currentLinVel - params.maxLinAccel*dt);
maxLin = min(params.maxLinVel, currentLinVel + params.maxLinAccel*dt);
minAng = max(-params.maxAngVel, currentAngVel - params.maxAngAccel*dt);
maxAng = min(params.maxAngVel, currentAngVel + params.maxAngAccel*dt);

bestScore = -inf;
bestLinVel = 0;
bestAngVel = 0;

% Distance to goal RIGHT NOW, before simulating anything — used to
% measure whether a candidate actually makes progress
distToGoalNow = norm(goal - pose(1:2));

for linVel = minLin:params.linVelStep:maxLin
    for angVel = minAng:params.angVelStep:maxAng

        simPose = pose;
        t = 0;
        collision = false;

        while t < predictTime
            simPose(1) = simPose(1) + linVel*cos(simPose(3))*dt;
            simPose(2) = simPose(2) + linVel*sin(simPose(3))*dt;
            simPose(3) = simPose(3) + angVel*dt;
            t = t + dt;

            if ~isempty(obstacles)
                dists = sqrt(sum((obstacles - simPose(1:2)).^2, 2));
                if any(dists < params.obstacleRadius + params.safetyMargin)
                    collision = true;
                    break;
                end
            end
        end

        if collision
            continue;
        end

        % --- Heading score (unchanged) ---
        goalVector = goal - simPose(1:2);
        headingToGoal = atan2(goalVector(2), goalVector(1));
        headingScore = pi - abs(wrapToPi(headingToGoal - simPose(3)));

        % --- Clearance score (unchanged) ---
        if ~isempty(obstacles)
            distToNearest = min(sqrt(sum((obstacles - simPose(1:2)).^2, 2)));
        else
            distToNearest = params.maxClearance;
        end
        clearanceScore = min(distToNearest, params.maxClearance);

        % --- Velocity score (unchanged) ---
        velocityScore = linVel;

        % --- NEW: Progress score — directly rewards closing the ---
        % --- distance to the goal, which "stand still and aim" ---
        % --- cannot win, since it makes zero progress ---
        distToGoalAfter = norm(goal - simPose(1:2));
        progressScore = distToGoalNow - distToGoalAfter;

        totalScore = params.headingWeight * headingScore + ...
            params.clearanceWeight * clearanceScore + ...
            params.velocityWeight * velocityScore + ...
            params.progressWeight * progressScore;

        if totalScore > bestScore
            bestScore = totalScore;
            bestLinVel = linVel;
            bestAngVel = angVel;
        end
    end
end

% Diagnostic print — now OFF by default, only prints if you explicitly
% set params.debug = true. Keeps Command Window clean during normal runs.
if isfield(params, 'debug') && params.debug
    fprintf('  [DWA] bestScore=%.2f | bestLin=%.2f | bestAng=%.2f\n', ...
        bestScore, bestLinVel, bestAngVel);
end
end