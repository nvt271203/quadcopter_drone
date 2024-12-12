% Startup script for project Quadcopter_Drone.prj
% Copyright 2018-2024 The MathWorks, Inc.

%% Code for building Simscape custom library at startup
% Change to folder with package directory
curr_proj = simulinkproject;
cd(curr_proj.RootFolder)

% Change to root folder
cd(curr_proj.RootFolder)

% If running in a parallel pool
% do not open model or demo script
open_start_content = 1;
if(~isempty(ver('parallel')))
    if(~isempty(getCurrentTask()))
        open_start_content = 0;
    end
end

if(open_start_content)
    % Parameters
    quadcopter_package_parameters;
    
    % Dữ liệu waypoints
    default_waypoints = [
        2   2    2   8   10   10;   % X
        0   0    4   0   0    0;    % Y
        0   5    5   5   5    0   % Z
    ];



    % tọa độ của chướng ngại vật
    obstacle_point = [
        5;    % X
        2;    % Y
        5     % Z
    ];




    avoidIt=true;
    roundtrip=false;

    % Define trajectory
    [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(default_waypoints, obstacle_point, avoidIt, roundtrip);
    % Set Python environment (if needed)
    check_pyenv
    % Open Model
    quadcopter_package_delivery % Not for Workshop
    % Open Exercises
    %quadcopter_workshop_prefs
    %quadcopter_drone_exercises_app_run  % For Workshop
end
