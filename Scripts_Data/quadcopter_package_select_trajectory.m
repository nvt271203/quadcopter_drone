function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(default_waypoints, obstacle_point, avoidIt, roundtrip)
%quadcopter_select_trajectory Obtain parameters for selected quadcopter trajectory
%   This function returns the essential parameters that define the
%   quadcopter's trajectory.

% Copyright 2021-2024 The MathWorks, Inc.

max_speed = 1;
min_speed = 0.1;
xApproach = [4 0.5];
vApproach = 0.1;
waypoints = default_waypoints;

% khúc xử lý
if avoidIt
    waypoints = avoid_obstacle(default_waypoints, obstacle_point);
end

disp(waypoints);
% phải có khúc này
% Tính toán spline và các thông số khác nếu cần
if exist("xApproach", "var")
    if roundtrip
        [timespot_spl_re, spline_data_re, spline_yaw_re, ~] = ...
            quadcopter_waypoints_to_trajectory(... 
            fliplr(waypoints), max_speed, min_speed, xApproach, vApproach);

        [timespot_spl_to, spline_data_to, spline_yaw_to, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(... 
            waypoints, max_speed, min_speed, xApproach, vApproach);

        pause_at_target = 5; % sec
        timespot_spl = [timespot_spl_to; timespot_spl_re + timespot_spl_to(end) + pause_at_target];

        spline_data = [spline_data_to; spline_data_re];
        spline_yaw = [spline_yaw_to spline_yaw_re];
        spline_yaw = unwrap(spline_yaw, 1.5 * pi);
    else
        [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(... 
            waypoints, max_speed, min_speed, xApproach, vApproach);
    end
else
    % Visualize the path between waypoints
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    if roundtrip
        spline_data = [spline_data; flipud(spline_data)];
        timespot_spl = [timespot_spl; timespot_spl(end) + 5; ...
            timespot_spl(end) + 5 + cumsum(flipud(diff(timespot_spl)))];
        spline_yaw = unwrap([spline_yaw flipud(spline_yaw) + pi], 1.5 * pi);
    end
end



