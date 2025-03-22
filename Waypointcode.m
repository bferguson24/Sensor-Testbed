x0 = 0;        % Initial x position
y0 = 1;        % Initial y position
angle_deg = 45; % Angle in degrees
horizontal_y = 0.5; % y position for the horizontal motion

[x_traj, y_traj] = generateTrajectory(x0, y0, angle_deg, horizontal_y);


function [x_traj, y_traj] = generateTrajectory(x0, y0, angle_deg, horizontal_y)
    % Convert angle to radians
    angle_rad = angle_deg * pi / 180;
    
    % Parameters
    bucket_radius = 0.05; % Radius from pivot to bucket tip (adjust as needed)
    y_horizontal = horizontal_y; % y position where the horizontal move happens
    x_end = 0.54; % Final x position
    num_points = 100; % Total points for even distribution
    
    % Downward motion
    y_final = y_horizontal; % Final y position before horizontal move
    x_final = x0 + (y0 - y_final) / tan(angle_rad);
    
    % Horizontal motion
    x_horizontal_end = x_end;
    
    % Upward motion (mirrored path)
    y_up_final = y0;
    x_up_final = x_horizontal_end + (y_up_final - y_final) / tan(angle_rad);
    
    % Generate points
    x_down = linspace(x0, x_final, num_points/3);
    y_down = y0 - tan(angle_rad) * (x_down - x0);
    
    x_horizontal = linspace(x_final, x_horizontal_end, num_points/3);
    y_horizontal = y_final * ones(size(x_horizontal));
    
    % Bucket pivot adjustment: Arc at transition
    theta_arc = linspace(pi, 1.5*pi, num_points/10); % 90-degree arc
    x_arc = x_final + bucket_radius * cos(theta_arc);
    y_arc = y_final + bucket_radius * (sin(theta_arc) + 1);
    
    x_up = linspace(x_horizontal_end, x_up_final, num_points/3);
    y_up = y_final + tan(angle_rad) * (x_up - x_horizontal_end);
    
    % Combine all segments
    x_traj = [x_down, x_arc, x_horizontal, x_up];
    y_traj = [y_down, y_arc, y_horizontal, y_up];
    
    % Plot the trajectory
    figure;
    plot(x_traj, y_traj, 'b-', 'LineWidth', 2); hold on;
    plot(x_traj, y_traj, 'ro', 'MarkerSize', 3);
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    title('Robot Bucket Tip Trajectory');
    grid on;
    axis equal;
end
