classdef Config
    % capstone.env.Config
    %  Nơi tập trung các tham số cấu hình chung cho App/Script.

    properties (Constant)
        % Thông số kết nối
        BtName    = "Behind the scream";
        BtChannel = 1;

        % Thông số robot (ví dụ)
        RobotSampleTime = 1/40;  % s

        % Giới hạn servo (có thể sửa theo thực tế)
        ServoMinDeg = 55;
        ServoMaxDeg = 115;
    end
end
