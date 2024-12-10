function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(path_line, varargin)
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

% Khởi tạo tọa độ khối vuông (Obstacle)
input_point = [0; 0; 6]; % Tọa độ góc dưới cùng bên trái của khối





% Tìm kiếm index của điểm trùng khớp
index = -1; % Giá trị mặc định nếu không tìm thấy
for i = 1:size(waypoints, 2)
    if isequal(waypoints(:, i), input_point)
        index = i;
        break;
    end
end

% Chuyển đổi index thành giá trị tương ứng cho switch-case
if path_line
    switch index
        case 1
            disp('Trùng với waypoint 1');
            % Gán thông số cho waypoint 1
            waypoints = [
                -1    -2    0   2    5;
                -3    -2    0   0    0;
                0.14  6    6   6   0.14
            ];
            max_speed = 1;
            min_speed = 0.1;
            xApproach = [4 0.5];
            vApproach = 0.1;

        case 2
            disp('Trùng với waypoint 2');
            waypoints = [
                  -2    -2    0   2    5;
                 -2    -2    0   0    0;
                  0.14  3    6   6   0.14
            ];
            
            % Gán thông số cho waypoint 2
            max_speed = 1.2;
            min_speed = 0.2;
            xApproach = [5 0.7];
            vApproach = 0.15;

        case 3
            disp('Trùng với waypoint 3');
            % Gán thông số cho waypoint 3
            waypoints = [
    -2    -2    0   2    5;
    -2    -2    0   0    0;
     0.14  6    8   6   0.14
];


            max_speed = 1.5;
            min_speed = 0.3;
            xApproach = [6 0.8];
            vApproach = 0.2;

        case 4
            disp('Trùng với waypoint 4');
                waypoints = [
            -2    -2    0   4    5;
            -2    -2    0   2    0;
             0.14  6    6   3   0.14
                                        ];
            % Gán thông số cho waypoint 4
            max_speed = 1.8;
            min_speed = 0.4;
            xApproach = [7 1];
            vApproach = 0.25;

        case 5
            disp('Trùng với waypoint 5');
            % Gán thông số cho waypoint 5
          waypoints = [
    -2    -2    0   2    7;
    -2    -2    0   0    0;
     0.14  6    6   6   0.2
];
            max_speed = 2;
            min_speed = 0.5;
            xApproach = [8 1.2];
            vApproach = 0.3;

        otherwise


           disp('Không trùng với waypoint nào. Default case.');
            % Giá trị mặc định
            waypoints = waypoints;
            max_speed = 0.5;
            min_speed = 0.1;
            xApproach = [2 0.2];
            vApproach = 0.05;
    end
else
    error('path_line phải là boolean true.');
end

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

end

% Hàm kiểm tra sự tồn tại của khối trong Simulink
function result = block_exists(blockName)
    % Hàm kiểm tra xem khối có tồn tại trong mô hình Simulink hay không
    result = ~isempty(find_system('Name', blockName));
end