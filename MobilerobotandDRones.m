%% Wheeled Medical Robot Simulation in a Care Home Environment
clc; clear; close all;

%% ==================== 1. Mechanical Design ====================
robot.radius = 0.4; % Radius of the robot (meters)
robot.max_speed = 1.5; % Maximum speed (m/s)
disp('=== Mechanical Design ===');
disp(['Robot Radius: ' num2str(robot.radius) ' m']);
disp(['Max Speed: ' num2str(robot.max_speed) ' m/s']);

%% ==================== 2. Environment Setup ====================
map = binaryOccupancyMap(100, 100, 1); % 100x100 grid, 1m resolution
inflate(map, 1); % Inflate obstacles for safety

% Define care home layout (walls, furniture, beds, etc.)
walls = [
    10 10; 10 90; 90 90; 90 10; % Outer boundary
    30 30; 30 70; 70 70; 70 30; % Room 1
    30 120; 30 160; 70 160; 70 120; % Room 2
];

furniture = [
    % Beds and tables in Room 1
    40 40; 50 40; 60 40;
    40 50; 50 50; 60 50;
    % Chairs in Room 2
    130 40; 140 40; 150 40;
];

% Combine walls and furniture into obstacles
all_obstacles = [walls; furniture];
setOccupancy(map, all_obstacles, 1);

%% ==================== 3. Visualization ====================
fig1 = figure('Name', 'Robot Navigation', 'Position', [100 100 800 700]);
ax1 = axes(fig1);
show(map, 'Parent', ax1);
hold(ax1, 'on');
grid(ax1, 'on');
ax1.GridAlpha = 0.3;

% Initialize start and goal markers (will be updated later)
startPlot = plot(ax1, 0, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'Visible', 'off');
goalPlot = plot(ax1, 0, 0, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'Visible', 'off');
title(ax1, 'Wheeled Medical Robot Navigation');
xlabel(ax1, 'X Coordinate (meters)');
ylabel(ax1, 'Y Coordinate (meters)');

%% ==================== 4. Path Planning ====================
ss = stateSpaceSE2([map.XWorldLimits; map.YWorldLimits; [-pi pi]]);
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 1;

planner = plannerRRTStar(ss, sv);
planner.MaxIterations = 3000;
planner.MaxConnectionDistance = 10; % Adjusted for indoor navigation
planner.GoalBias = 0.3;

%% ==================== 5. Generate Valid Random Start/Goal Positions ====================
% Generate random start and goal positions
start_pos = findFreePosition(map);
goal_pos = findFreePosition(map);

% Ensure minimum separation between start and goal
while norm(start_pos - goal_pos) < 30 % Minimum distance of 30 meters
    goal_pos = findFreePosition(map);
end

% Update the start and goal markers with actual positions
set(startPlot, 'XData', start_pos(1), 'YData', start_pos(2), 'Visible', 'on');
set(goalPlot, 'XData', goal_pos(1), 'YData', goal_pos(2), 'Visible', 'on');

%% ==================== 6. Plan Path ====================
start = [start_pos, 0]; % Start with orientation
goal = [goal_pos, 0]; % Goal with orientation
[pthObj, solnInfo] = plan(planner, start, goal);

if ~solnInfo.IsPathFound
    path = [linspace(start(1), goal(1), 100)', linspace(start(2), goal(2), 100)', zeros(100, 1)];
    warning('Using straight-line fallback path');
else
    path = pthObj.States;
end

% Plot planned path
pathPlot = plot(ax1, path(:, 1), path(:, 2), 'g-', 'LineWidth', 2);

%% ==================== 7. Robot Simulation ====================
currentPos = start_pos;
traveledPath = currentPos;

% Create detailed robot representation
robotBody = rectangle(ax1, 'Position', [currentPos(1)-robot.radius, currentPos(2)-robot.radius, 2*robot.radius, 2*robot.radius], ...
    'Curvature', [1, 1], 'FaceColor', 'b', 'EdgeColor', 'k', 'LineWidth', 2);

% Create traveled path plot
travelPlot = plot(ax1, traveledPath(:, 1), traveledPath(:, 2), 'b-', 'LineWidth', 1.5);

% Simulation parameters
dt = 0.2; % Timestep
animation_speed = 0.1; % Pause time for smoother animation

% Interpolate path for smoother animation
interp_factor = 10;
xi = interp1(1:size(path, 1), path(:, 1), linspace(1, size(path, 1), size(path, 1)*interp_factor))';
yi = interp1(1:size(path, 1), path(:, 2), linspace(1, size(path, 1), size(path, 1)*interp_factor))';
interp_path = [xi yi];

% Run simulation
for i = 1:size(interp_path, 1)
    try
        % Update robot position
        newPos = interp_path(i, :);
        currentPos = newPos;
        traveledPath = [traveledPath; currentPos];
        
        % Update robot visualization
        set(robotBody, 'Position', [currentPos(1)-robot.radius, currentPos(2)-robot.radius, 2*robot.radius, 2*robot.radius]);
        set(travelPlot, 'XData', traveledPath(:, 1), 'YData', traveledPath(:, 2));
        
        drawnow;
        pause(animation_speed);
    catch ME
        warning('Error at step %d: %s', i, ME.message);
        break;
    end
end

% Finalize plots
legend(ax1, [startPlot, goalPlot, pathPlot, travelPlot], ...
    {'Start', 'Goal', 'Planned Path', 'Traveled Path'}, ...
    'Location', 'northeastoutside', 'FontSize', 8);
title(ax1, 'Mission Complete');

%% ==================== Local Function Definitions ====================
function pos = findFreePosition(map)
    while true
        pos = [randi([map.XWorldLimits(1)+5, map.XWorldLimits(2)-5]), ...
               randi([map.YWorldLimits(1)+5, map.YWorldLimits(2)-5])];
        if ~checkOccupancy(map, pos)
            break;
        end
    end
end