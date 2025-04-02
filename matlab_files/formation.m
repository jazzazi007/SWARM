clear all; close all; clc;

% Initial positions of the swarm (3 points)
points = [-0.5, 0.5; -0.7, 0.3; -0.3, 0.3]; % [x, y] for each point in V-formation
goal = [3.5, 2.75]; % Goal position
position_accuracy = 0.05;

% APF parameters
zeta = 1.1547;
eta = 0.0732;
dstar = 0.3;
Qstar = 0.75;

% Tunnel details
tunnel_entry = [1.5, 1.5];
tunnel_exit = [2.5, 2.5];
tunnel_radius = 0.3; % Radius around which the tunnel detection triggers
tunnel_path = [linspace(tunnel_entry(1), tunnel_exit(1), 100); linspace(tunnel_entry(2), tunnel_exit(2), 100)];

% Kinematic model parameters
v_max = 0.2;
Kp_omega = 1.5;
omega_max = 0.5 * pi;
error_theta_max = deg2rad(45);

% Formation parameters
v_distance = 0.3; % Distance for V-formation
seq_distance = 0.2; % Distance for sequential formation

% Initialize swarm formation mode
formation_mode = 'V';

% Initialize orientations for each point
thetas = zeros(size(points, 1), 1); % Store orientation for each point

% Simulation parameters
t = 1;
dT = 0.1;
t_max = 1000;
X = cell(1, t_max); % Stores X positions for all points over time
Y = cell(1, t_max); % Stores Y positions for all points over time

% Main loop
while norm(goal - points(1, :)) > position_accuracy && t < t_max
    % Check if in tunnel detection range
    dist_to_tunnel = norm(points(1, :) - tunnel_entry);
    if dist_to_tunnel < tunnel_radius
        formation_mode = 'Sequence';
    elseif norm(points(1, :) - tunnel_exit) < tunnel_radius
        formation_mode = 'V';
    end

    % Update each mass point's position based on the formation mode
    for i = 1:size(points, 1)
        % Define the goal for each point in the current formation
        if strcmp(formation_mode, 'V')
            % Set V-formation target for each point relative to the leader
            if i == 1
                target = goal; % Leader moves toward goal
            else
                angle_offset = pi / 6 * (-1) ^ i; % Angle offset for V-formation
                target = points(1, :) + v_distance * [cos(angle_offset), sin(angle_offset)];
            end
        elseif strcmp(formation_mode, 'Sequence')
            % Set sequential target for each point relative to the previous point
            if i == 1
                target = goal; % Leader moves toward goal
            else
                target = points(i - 1, :) - seq_distance * [cos(thetas(i - 1)), sin(thetas(i - 1))];
            end
        end

        % Calculate Attractive Potential
        if norm(points(i, :) - target) <= dstar
            nablaU_att = zeta * (points(i, :) - target);
        else
            nablaU_att = dstar / norm(points(i, :) - target) * zeta * (points(i, :) - target);
        end

        % Repulsive Potential (only for leader)
        nablaU_rep = [0 0];
        if i == 1
            % Calculate distance to the tunnel walls as repulsive fields
            [~, obst1_dist] = dsearchn(tunnel_path', points(i, :));
            if obst1_dist <= Qstar
                nablaU_rep = eta * (1 / Qstar - 1 / obst1_dist) * 1 / obst1_dist^2 * ...
                             (points(i, :) - tunnel_path(:, round(obst1_dist * 100))');
            end
        end

        % Calculate combined potential
        nablaU = nablaU_att + nablaU_rep;

        % Calculate reference values for velocity and orientation
        theta_ref = atan2(-nablaU(2), -nablaU(1));
        error_theta = theta_ref - thetas(i);
        if abs(error_theta) <= error_theta_max
            alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
            v_ref = min(alpha * norm(-nablaU), v_max);
        else
            v_ref = 0;
        end

        % Update kinematics
        omega_ref = Kp_omega * error_theta;
        omega_ref = min(max(omega_ref, -omega_max), omega_max);
        thetas(i) = thetas(i) + omega_ref * dT; % Update orientation for each point
        points(i, 1) = points(i, 1) + v_ref * cos(thetas(i)) * dT;
        points(i, 2) = points(i, 2) + v_ref * sin(thetas(i)) * dT;

        % Store positions
        X{t}(i) = points(i, 1);
        Y{t}(i) = points(i, 2);
    end

    % Plotting
    cla;
    daspect([1 1 1]);
    xlim([-1, 4]);
    ylim([-1, 4]);
    box on; hold on;
    plot(tunnel_path(1, :), tunnel_path(2, :), '-r', 'LineWidth', 1.5); % Tunnel path
    plot(goal(1), goal(2), 'ob'); % Goal position

    % Plot coverage area as circle around points
    coverage_radius = 0.6; % Example coverage radius
    viscircles(points, coverage_radius * ones(size(points, 1), 1), 'LineStyle', '--');

    % Plot swarm points and paths
    for i = 1:size(points, 1)
        plot(X{t}(i), Y{t}(i), 'ok', 'MarkerFaceColor', 'k'); % Current positions
    end
    for k = 1:t
        plot(cell2mat(X(k)), cell2mat(Y(k)), '-b'); % Trajectory path
    end
    drawnow;
    pause(dT);
    
    t = t + 1;
end
disp("Simulation complete.");
