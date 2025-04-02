clear all; close all; clc;

% Initial positions and orientations for 3 mass points (forming a V shape)
x = [-0.5, -0.5 4];
y = [1, 0.8, 0.5];
theta = [0, 0, 0];

% Goal position
x_goal = 3.5;
y_goal = 2.75;
position_accuracy = 0.05;

% APF parameters
zeta = 1.1547;
eta = 0.0732;
dstar = 0.3;
Qstar = 0.75;

% Parameters related to kinematic model
error_theta_max = deg2rad(45);
v_max = 0.2;
Kp_omega = 1.5;
omega_max = 0.5 * pi;

% Obstacles setup
obst1_points = [linspace(1.5, 1.5, 100) linspace(1.5, 1.5, 100) linspace(2, 2, 100) linspace(2, 1.5, 100);
                linspace(1.5, 2, 100) linspace(2, 2, 100) linspace(2, 1.5, 100) linspace(1.5, 1.5, 100)] - 1;
obst2_points = [2 + sin(linspace(0, pi / 2, 100)) linspace(3, 3, 100) linspace(3, 2, 100);
                2.5 + cos(linspace(0, pi / 2, 100)) linspace(2.5, 3.5, 100) linspace(3.5, 3.5, 100)] - [0; 1.5];

% V-formation parameters
v_distance = 0.3;         % Distance to maintain in V-formation
v_angle = deg2rad(45);    % Angle of V-formation

figure(1);
dT = 0.1;
t_max = 1000;

% Initialize path storage for each point
X = zeros(3, t_max);
Y = zeros(3, t_max);
X(:, 1) = x;
Y(:, 1) = y;

% Initialize Lyapunov function history
V_history = zeros(1, t_max);

t = 1;
while t <= t_max
    all_reached_goal = true;

    % Lyapunov function calculation
    V_leader = 0.5 * ((x(1) - x_goal)^2 + (y(1) - y_goal)^2); % Leader energy
    V_follower1 = 0.5 * ((x(2) - (x(1) - v_distance * cos(theta(1) + v_angle)))^2 + ...
                         (y(2) - (y(1) - v_distance * sin(theta(1) + v_angle)))^2); % Follower 1
    V_follower2 = 0.5 * ((x(3) - (x(1) - v_distance * cos(theta(1) - v_angle)))^2 + ...
                         (y(3) - (y(1) - v_distance * sin(theta(1) - v_angle)))^2); % Follower 2
    V_total = V_leader + V_follower1 + V_follower2; % Total Lyapunov function

    V_history(t) = V_total; % Store for plotting

    for i = 1:3
        if i == 1
            % Leader behavior
            if norm([x_goal, y_goal] - [x(1), y(1)]) > position_accuracy
                all_reached_goal = false;

                % Calculate Attractive Potential
                if norm([x(1), y(1)] - [x_goal, y_goal]) <= dstar
                    nablaU_att = zeta * ([x(1), y(1)] - [x_goal, y_goal]);
                else
                    nablaU_att = (dstar / norm([x(1), y(1)] - [x_goal, y_goal])) * zeta * ([x(1), y(1)] - [x_goal, y_goal]);
                end

                % Calculate distances to obstacles
                [obst1_idx, obst1_dist] = dsearchn(obst1_points', [x(1), y(1)]);
                [obst2_idx, obst2_dist] = dsearchn(obst2_points', [x(1), y(1)]);

                % Calculate Repulsive Potential
                nablaU_rep = [0, 0];
                if obst1_dist <= Qstar
                    nablaU_rep = nablaU_rep + (eta * (1 / Qstar - 1 / obst1_dist) * 1 / obst1_dist^2) * ([x(1), y(1)] - [obst1_points(1, obst1_idx), obst1_points(2, obst1_idx)]);
                end
                if obst2_dist <= Qstar && ~inpolygon(x(1), y(1), obst2_points(1, :), obst2_points(2, :))
                    nablaU_rep = nablaU_rep + (eta * (1 / Qstar - 1 / obst2_dist) * 1 / obst2_dist^2) * ([x(1), y(1)] - [obst2_points(1, obst2_idx), obst2_points(2, obst2_idx)]);
                end

                % Calculate final potential
                nablaU = nablaU_att + nablaU_rep;

                % Calculate reference velocity (v_ref) and orientation (theta_ref)
                theta_ref = atan2(-nablaU(2), -nablaU(1));
                error_theta = theta_ref - theta(1);

                if abs(error_theta) <= error_theta_max
                    alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
                    v_ref = min(alpha * norm(-nablaU), v_max);
                else
                    v_ref = 0;
                end

                % Update kinematic model for leader
                omega_ref = Kp_omega * error_theta;
                omega_ref = min(max(omega_ref, -omega_max), omega_max);
                
                theta(1) = theta(1) + omega_ref * dT;
                x(1) = x(1) + v_ref * cos(theta(1)) * dT;
                y(1) = y(1) + v_ref * sin(theta(1)) * dT;
            end
        else
            % Follower behavior
            if i == 2
                angle_offset = v_angle;
            else
                angle_offset = -v_angle;
            end

            formation_x = x(1) - v_distance * cos(theta(1) + angle_offset);
            formation_y = y(1) - v_distance * sin(theta(1) + angle_offset);

            if norm([formation_x, formation_y] - [x(i), y(i)]) > position_accuracy
                all_reached_goal = false;

                % Calculate Attractive Potential
                if norm([x(i), y(i)] - [formation_x, formation_y]) <= dstar
                    nablaU_att = zeta * ([x(i), y(i)] - [formation_x, formation_y]);
                else
                    nablaU_att = (dstar / norm([x(i), y(i)] - [formation_x, formation_y])) * zeta * ([x(i), y(i)] - [formation_x, formation_y]);
                end

                % Calculate Repulsive Potential
                [obst1_idx, obst1_dist] = dsearchn(obst1_points', [x(i), y(i)]);
                [obst2_idx, obst2_dist] = dsearchn(obst2_points', [x(i), y(i)]);

                nablaU_rep = [0, 0];
                if obst1_dist <= Qstar
                    nablaU_rep = nablaU_rep + (eta * (1 / Qstar - 1 / obst1_dist) * 1 / obst1_dist^2) * ([x(i), y(i)] - [obst1_points(1, obst1_idx), obst1_points(2, obst1_idx)]);
                end
                if obst2_dist <= Qstar && ~inpolygon(x(i), y(i), obst2_points(1, :), obst2_points(2, :))
                    nablaU_rep = nablaU_rep + (eta * (1 / Qstar - 1 / obst2_dist) * 1 / obst2_dist^2) * ([x(i), y(i)] - [obst2_points(1, obst2_idx), obst2_points(2, obst2_idx)]);
                end

                % Combine Potentials
                nablaU = nablaU_att + nablaU_rep;

                % Calculate reference velocity (v_ref) and orientation (theta_ref)
                theta_ref = atan2(-nablaU(2), -nablaU(1));
                error_theta = theta_ref - theta(i);

                if abs(error_theta) <= error_theta_max
                    alpha = (error_theta_max - abs(error_theta)) / error_theta_max;
                    v_ref = min(alpha * norm(-nablaU), v_max);
                else
                    v_ref = 0;
                end

                % Update kinematic model for followers
                omega_ref = Kp_omega * error_theta;
                omega_ref = min(max(omega_ref, -omega_max), omega_max);

                theta(i) = theta(i) + omega_ref * dT;
                x(i) = x(i) + v_ref * cos(theta(i)) * dT;
                y(i) = y(i) + v_ref * sin(theta(i)) * dT;
            end
        end
    end

    % Stop if all points reached the goal
    if all_reached_goal
        break;
    end

    % Store and plot paths
    X(:, t) = x;
    Y(:, t) = y;

    cla;
    daspect([1, 1, 1]);
    xlim([-1, 4]);
    ylim([-1, 3]);
    box on;
    hold on;
    plot(obst1_points(1, :), obst1_points(2, :), '-r');
    plot(obst2_points(1, :), obst2_points(2, :), '-r');
    plot(x_goal, y_goal, 'ob');
    
    % Plot formation area
    fill(x, y, 'cyan', 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Coverage area

    for i = 1:3
        plot(X(i, 1:t), Y(i, 1:t), '-b');
        plot([x(i), x(i) + 0.2 * cos(theta(i))], [y(i), y(i) + 0.2 * sin(theta(i))], '-r'); % Orientation
    end

    drawnow;
    pause(dT);
    t = t + 1;
end

% Plot Lyapunov function
figure;
plot(1:t, V_history(1:t));
title('Lyapunov Function Over Time');
xlabel('Time Step');
ylabel('Total Energy (V_{total})');
grid on;

disp("Travel time: " + (t * dT) + " seconds.");
