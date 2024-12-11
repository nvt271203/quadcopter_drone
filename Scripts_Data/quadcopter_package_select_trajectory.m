function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis, input_point] = quadcopter_package_select_trajectory(path_line, varargin)
%quadcopter_select_trajectory Obtain parameters for selected quadcopter trajectory
%   This function returns the essential parameters that define the
%   quadcopter's trajectory.

% Copyright 2021-2024 The MathWorks, Inc.

if nargin == 2
    roundtrip = varargin{1};
else
    roundtrip = false;
end


% Dữ liệu waypoints
waypoints = [
    -2    -2    0   2    5;
    -2    -2    0   0    0;
     0.14  6    6   6   0.14
];

input_point = [3.5; 0; 3.07]; % KHởi tạo tọa độ của chướng ngại vật

max_speed = 1;
min_speed = 0.1;
xApproach = [4 0.5];
vApproach = 0.1;

% khúc xử lý
waypoints = avoid_obstacle(waypoints, input_point);

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



