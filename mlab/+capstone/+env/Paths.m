classdef Paths < handle
    % capstone.env.Paths
    %  - Xác định root repo, thư mục mlab, low_level
    %  - Thiết lập addpath cho các thư mục cần thiết
    %  - Lưu lại path cũ để có thể restore

    properties (SetAccess = private)
        RepoRoot      (1,1) string
        MlabRoot      (1,1) string
        LowLevelRoot  (1,1) string
        AppFolder     (1,1) string
        ScriptsFolder (1,1) string
        LibFolder     (1,1) string

        OriginalPath  (1,1) string
        IsOnPath      (1,1) logical = false
    end

    methods
        function obj = Paths()
            % Xác định vị trí file Paths.m
            thisFile = mfilename('fullpath');              % .../mlab/+capstone/+env/Paths.m
            envFolder = fileparts(thisFile);               % .../mlab/+capstone/+env
            capstonePkg = fileparts(envFolder);            % .../mlab/+capstone
            mlabRoot   = fileparts(capstonePkg);           % .../mlab
            repoRoot   = fileparts(mlabRoot);              % .../capstone_smartLineFollowingRobot-

            obj.MlabRoot      = string(mlabRoot);
            obj.RepoRoot      = string(repoRoot);
            obj.LowLevelRoot  = string(fullfile(repoRoot, 'low_level'));
            obj.AppFolder     = string(fullfile(mlabRoot, 'app'));
            obj.ScriptsFolder = string(fullfile(mlabRoot, 'scripts'));
            obj.LibFolder     = string(fullfile(mlabRoot, 'lib'));

            % Tự động addpath khi tạo object
            obj.addAll();
        end

        function addAll(obj)
            % Lưu path hiện tại (chỉ lần đầu)
            if ~obj.IsOnPath
                obj.OriginalPath = string(path());
            end

            % Thêm mlab root
            addpath(obj.MlabRoot);

            % Thêm scripts (nếu tồn tại)
            if isfolder(obj.ScriptsFolder)
                addpath(genpath(obj.ScriptsFolder));
            end

            % Thêm lib (nếu tồn tại)
            if isfolder(obj.LibFolder)
                addpath(genpath(obj.LibFolder));
            end

            % Thêm app folder để MATLAB thấy .mlapp (không bắt buộc, nhưng tiện)
            if isfolder(obj.AppFolder)
                addpath(obj.AppFolder);
            end

            obj.IsOnPath = true;
        end

        function restore(obj)
            % Khôi phục path về trạng thái trước khi addAll()
            if obj.IsOnPath && strlength(obj.OriginalPath) > 0
                path(char(obj.OriginalPath));
                obj.IsOnPath = false;
            end
        end
    end
end
