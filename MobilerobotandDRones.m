% Simple Care Home Robot Navigation with A* and Obstacle Avoidance
clc; clear; close all;

%% 1. Setup Environment
grid_size = 20; % 20x20m environment
resolution = 1; % 1 cell/meter
map = binaryOccupancyMap(grid_size, grid_size, resolution);
inflate(map, 0.5); % Inflate obstacles for safety

% Add static obstacles (walls, furniture)
obstacles = [
    2 2; 3 2; 4 2; 5 2; 6 2; 7 2;
    5 3; 5 4; 5 5; 5 6;
    8 8; 8 7; 8 6;
    3 8; 4 8; 3 9; 4 9;
    15 15; 15 16; 16 15; 16 16;
    10 5; 11 5; 12 5; 13 5;
];
setOccupancy(map, obstacles, 1);

%% 2. Set Start and Goal Positions
start_pos = [2, 3];
goal_pos = [18, 17];

if checkOccupancy(map, start_pos) || checkOccupancy(map, goal_pos)
    error('Start or goal position inside obstacle!');
end

%% 3. Path Planning (A*)
planner = plannerAStarGrid(map);

% Convert world -> grid for planning
start_grid = world2grid(map, start_pos);
goal_grid = world2grid(map, goal_pos);

path = plan(planner, start_grid, goal_grid);
path = grid2world(map, path);

%% 4. Visualization Setup
fig = figure('Name', 'Simple Navigation with Obstacle Avoidance');
ax = axes('Parent', fig);
show(map, 'Parent', ax); 
hold(ax, 'on');

% Initialize all plot handles
h_start = plot(ax, start_pos(1), start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal = plot(ax, goal_pos(1), goal_pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
h_path = plot(ax, path(:,1), path(:,2), 'b-', 'LineWidth', 2);
h_robot = plot(ax, start_pos(1), start_pos(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
h_traj = plot(ax, start_pos(1), start_pos(2), 'm-', 'LineWidth', 1.5);
legend([h_start, h_goal, h_path, h_robot, h_traj], ...
    {'Start','Goal','Global Path','Robot','Trajectory'});

%% 5. Navigation Loop with Obstacle Avoidance
robotPose = [start_pos, 0]; % [x, y, theta]
trajectory = robotPose(1:2);
safe_distance = 0.8;
max_attempts = 1000;
attempt = 0;
reached_goal = false;

% Variables for stuck detection
last_position = start_pos; % Track the robot's last position
stuck_threshold = 0.1;     % Distance threshold to detect being stuck
stuck_count = 0;           % Counter for consecutive stuck iterations
max_stuck_attempts = 10;   % Maximum allowed stuck iterations

% Main navigation loop
while attempt < max_attempts && ~reached_goal
    attempt = attempt + 1;
    
    % Check if the robot is stuck
    if norm(robotPose(1:2) - last_position) < stuck_threshold
        stuck_count = stuck_count + 1;
    else
        stuck_count = 0; % Reset stuck counter if the robot moves
    end
    
    % Handle being stuck
    if stuck_count > max_stuck_attempts
        warning('Robot is stuck! Attempting to escape...');
        % Randomly perturb the robot's position to break out of the local minimum
        robotPose(1:2) = robotPose(1:2) + randn(1, 2) * 0.5; % Small random movement
        stuck_count = 0; % Reset stuck counter
    end
    
    % Update last position
    last_position = robotPose(1:2);
    
    % Check for obstacles near the robot
    [obstacle_detected, ~] = check_local_obstacles(map, robotPose, safe_distance);
    
    % If obstacle detected, replan path from current position
    if obstacle_detected
        start_grid = world2grid(map, robotPose(1:2));
        new_path = plan(planner, start_grid, goal_grid);
        
        if ~isempty(new_path)
            path = grid2world(map, new_path);
            % Update path plot
            set(h_path, 'XData', path(:,1), 'YData', path(:,2));
        else
            warning('No valid path found!');
            break;
        end
    end
    
    % Move toward the next point in the path
    if size(path,1) > 1
        target_pt = path(2,:); % Always go to next point in path
        [v, omega] = simple_controller(robotPose, target_pt);
        robotPose = update_pose(robotPose, v, omega, 0.1);
        
        % Remove passed points from path
        if norm(robotPose(1:2) - path(1,:)) < 0.3
            path(1,:) = [];
        end
    end
    
    % Update visualization
    trajectory = [trajectory; robotPose(1:2)];
    
    % Update robot position
    set(h_robot, 'XData', robotPose(1), 'YData', robotPose(2));
    
    % Update trajectory
    set(h_traj, 'XData', trajectory(:,1), 'YData', trajectory(:,2));
    
    drawnow;
    
    % Check if goal reached
    if norm(robotPose(1:2) - goal_pos) < 0.5
        reached_goal = true;
        disp('Goal reached successfully!');
    end
    
    pause(0.05);
    
    % Check if figure is still open
    if ~isvalid(fig)
        disp('Figure closed by user - stopping navigation');
        break;
    end
end

if ~reached_goal && isvalid(fig)
    disp('Navigation terminated - maximum attempts reached');
end

%% Helper Functions
function [obstacle_detected, obstacle_pos] = check_local_obstacles(map, pose, safe_dist)
    % Improved obstacle detection in front of the robot
    robot_pos = pose(1:2);
    theta = pose(3);
    
    % Check points in an arc in front of the robot
    num_check_points = 7; % More points for better coverage
    angles = linspace(-pi/3, pi/3, num_check_points); % Arc spanning -60° to +60°
    check_dist = safe_dist * 1.5;
    check_pts = [];
    
    for i = 1:num_check_points
        check_pts = [check_pts; ...
            robot_pos(1) + check_dist*cos(theta + angles(i)), ...
            robot_pos(2) + check_dist*sin(theta + angles(i))];
    end
    
    % Check occupancy
    occ_values = checkOccupancy(map, check_pts);
    
    if any(occ_values > 0)
        obstacle_detected = true;
        obstacle_pos = check_pts(find(occ_values > 0, 1), :);
    else
        obstacle_detected = false;
        obstacle_pos = [NaN, NaN];
    end
end

function [v, omega] = simple_controller(pose, target)
    % PID controller for improved performance
    Kp_v = 0.5;  % Linear velocity gain
    Ki_v = 0.01; % Integral gain for linear velocity
    Kd_v = 0.1;  % Derivative gain for linear velocity
    
    Kp_w = 1.0;  % Angular velocity gain
    Ki_w = 0.02; % Integral gain for angular velocity
    Kd_w = 0.2;  % Derivative gain for angular velocity
    
    persistent prev_dx prev_dy prev_angle_error integral_v integral_w
    if isempty(prev_dx)
        prev_dx = 0;
        prev_dy = 0;
        prev_angle_error = 0;
        integral_v = 0;
        integral_w = 0;
    end
    
    % Calculate errors
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);
    distance_error = sqrt(dx^2 + dy^2);
    angle_error = atan2(dy, dx) - pose(3);
    angle_error = mod(angle_error + pi, 2*pi) - pi; % Normalize to [-pi, pi]
    
    % Update integrals
    integral_v = integral_v + distance_error;
    integral_w = integral_w + angle_error;
    
    % Calculate derivatives
    deriv_v = distance_error - sqrt(prev_dx^2 + prev_dy^2);
    deriv_w = angle_error - prev_angle_error;
    
    % Calculate control outputs
    v = Kp_v * distance_error + Ki_v * integral_v + Kd_v * deriv_v;
    omega = Kp_w * angle_error + Ki_w * integral_w + Kd_w * deriv_w;
    
    % Limit velocities
    v = min(max(v, -0.5), 0.5);
    omega = min(max(omega, -1.0), 1.0);
    
    % Update persistent variables
    prev_dx = dx;
    prev_dy = dy;
    prev_angle_error = angle_error;
end

function new_pose = update_pose(pose, v, omega, dt)
    % Simple pose update with differential drive kinematics
    new_pose = pose;
    if abs(omega) < 1e-6
        % Straight line motion
        new_pose(1) = pose(1) + v * cos(pose(3)) * dt;
        new_pose(2) = pose(2) + v * sin(pose(3)) * dt;
    else
        % Circular motion
        radius = v / omega;
        new_pose(1) = pose(1) + radius * (sin(pose(3) + omega*dt) - sin(pose(3)));
        new_pose(2) = pose(2) - radius * (cos(pose(3) + omega*dt) - cos(pose(3)));
        new_pose(3) = pose(3) + omega * dt;
    end
end