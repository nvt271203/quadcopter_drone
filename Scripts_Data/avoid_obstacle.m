function updated_waypoints = avoid_obstacle(waypoints, input_point)
    space_x = 1;
    space_y = 1;
    % Duyệt qua từng đoạn đường
    for i = 1:(size(waypoints, 2) - 1)
        p1 = waypoints(:, i);      % Điểm đầu
        p2 = waypoints(:, i + 1);  % Điểm cuối
        
        v1 = p2 - p1;              % Vector đoạn thẳng
        v2 = input_point - p1;     % Vector từ p1 tới điểm cần kiểm tra
        
        % Kiểm tra cùng phương
        if norm(cross(v1, v2)) < 1e-6 % Tích có hướng gần bằng 0
            t = dot(v2, v1) / dot(v1, v1); % Tính t
            if t >= 0 && t <= 1 % Kiểm tra t trong đoạn [0, 1]
                % Xác định điểm cắt
                intersection_point = p1 + t * v1;
                
                % Kiểm tra nếu điểm cắt trùng với một điểm trong waypoints
                for j = 1:size(waypoints, 2)
                    if norm(intersection_point - waypoints(:, j)) < 1e-6
                        % Nếu trùng, xóa điểm
                        waypoints(:, j) = [];
                        break;
                    end
                end
                
                % Xác định điểm A và B cách điểm cắt 3 đơn vị trên đường thẳng
                direction = (p2 - p1) / norm(p2 - p1); % Hướng của đoạn thẳng
                A = intersection_point - space_x * direction;
                B = intersection_point + space_x * direction;

                % Tạo điểm C và D từ điểm A và B, dịch chuyển sang phải 4 đơn vị
                right_direction = cross(direction, [0; 0; 1]); % Vuông góc (trong không gian 3 chiều)
                right_direction = right_direction / norm(right_direction); % Đơn vị hóa
                C = A + space_y * right_direction;
                D = B + space_y * right_direction;

                % Cập nhật waypoints
                waypoints = [waypoints(:, 1:i), A, C, D, B, waypoints(:, (i + 2):end)];
                break;
            end
        end
    end
    
    updated_waypoints = waypoints; % Trả về đường đi sau khi xử lý
end
