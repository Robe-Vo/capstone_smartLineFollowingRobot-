function paths = setup()
% capstone.setup  Khởi tạo project environment cho MATLAB App.
%
% Cách dùng:
%   >> paths = capstone.setup;
%   % trả về đối tượng Paths, đã addpath đầy đủ

    paths = capstone.env.Paths();  % tạo object, auto addpath trong constructor

    % Có thể in ra 1-2 thông tin cơ bản (nếu muốn):
    % fprintf('Repo root: %s\n', paths.RepoRoot);
end
