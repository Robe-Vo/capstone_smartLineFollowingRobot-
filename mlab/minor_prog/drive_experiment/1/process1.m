%% 1. Load dữ liệu từ các file rampResult<i>.mat

fileNames = { ...
    'rampResult1.mat', ...
    'rampResult2.mat', ...
    'rampResult3.mat', ...
    'rampResult4.mat'};

nExp   = numel(fileNames);
expAll = cell(1,nExp);

for k = 1:nExp
    S  = load(fileNames{k});          % load struct trong file
    fn = fieldnames(S);               % lấy tên biến bên trong
    expAll{k} = S.(fn{1});            % lưu struct vào cell
end

% Tham số người dùng
nIdleSamples = 30;    % số mẫu đầu để ước lượng nhiễu
kNoiseStd    = 3;     % hệ số nhân std để đặt ngưỡng rpm
showFigures  = true;  % bật/tắt vẽ hình


%% 2. Kiểm tra ramp theo thời gian (speed_cmd và encoder_rpm)

if showFigures
    for k = 1:nExp
        data = expAll{k};

        t = data.t(:);
        u = data.speed_cmd(:);
        y = data.encoder_rpm(:);

        figure('Name',sprintf('Exp %d - ramp time plot',k));
        yyaxis left
        plot(t,u,'-'); grid on
        ylabel('speed\_cmd')
        yyaxis right
        plot(t,y,'-');
        ylabel('encoder\_rpm')
        xlabel('Time [s]')
        title(sprintf('Experiment %d: speed\\_cmd & encoder\\_rpm',k));
    end
end


%% 3. Đặc tính tĩnh encoder_rpm theo speed_cmd

if showFigures
    for k = 1:nExp
        data = expAll{k};

        u = data.speed_cmd(:);
        y = data.encoder_rpm(:);

        figure('Name',sprintf('Exp %d - static characteristic',k));
        plot(u,y,'.'); grid on
        xlabel('speed\_cmd')
        ylabel('encoder\_rpm')
        title(sprintf('Experiment %d: encoder\\_rpm vs speed\\_cmd',k));
    end
end


%% 4. Ước lượng nhiễu ở trạng thái đứng yên cho từng thí nghiệm

noiseStats = struct('yIdleMax',[],'yIdleStd',[],'rpmThresh',[]);

for k = 1:nExp
    data = expAll{k};
    y    = data.encoder_rpm(:);

    y_idle = y(1:min(nIdleSamples,numel(y)));

    noiseStats(k).yIdleMax  = max(abs(y_idle));
    noiseStats(k).yIdleStd  = std(y_idle,1);
    noiseStats(k).rpmThresh = noiseStats(k).yIdleMax + kNoiseStd*noiseStats(k).yIdleStd;
end


%% 5. Tìm deadband cho từng thí nghiệm (chiều dương / âm nếu có)

deadbandPerExp = struct( ...
    'u_db_pos',NaN,'t_db_pos',NaN, ...
    'u_db_neg',NaN,'t_db_neg',NaN);

for k = 1:nExp
    data = expAll{k};

    t   = data.t(:);
    u   = data.speed_cmd(:);
    y   = data.encoder_rpm(:);
    thr = noiseStats(k).rpmThresh;

    % Deadband phía dương: điểm đầu tiên y > thr và u > 0
    idxPos = find( (y >  thr) & (u > 0), 1, 'first');
    if ~isempty(idxPos)
        deadbandPerExp(k).u_db_pos = u(idxPos);
        deadbandPerExp(k).t_db_pos = t(idxPos);
    end

    % Deadband phía âm (nếu có ramp âm): y < -thr và u < 0
    idxNeg = find( (y < -thr) & (u < 0), 1, 'first');
    if ~isempty(idxNeg)
        deadbandPerExp(k).u_db_neg = u(idxNeg);
        deadbandPerExp(k).t_db_neg = t(idxNeg);
    end
end


%% 6. Tổng hợp deadband, phát hiện ngoại lai, tính deadband robust (median)

u_db_pos_all = [deadbandPerExp.u_db_pos];
u_db_neg_all = [deadbandPerExp.u_db_neg];

% Giá trị hợp lệ (không NaN)
pos_valid = u_db_pos_all(~isnan(u_db_pos_all));
neg_valid = u_db_neg_all(~isnan(u_db_neg_all));

% Tham số phát hiện ngoại lai (IQR)
kOut = 1.5;

% Phía dương
if ~isempty(pos_valid)
    Q1_pos = quantile(pos_valid,0.25);
    Q3_pos = quantile(pos_valid,0.75);
    IQR_pos = Q3_pos - Q1_pos;
    lower_pos = Q1_pos - kOut*IQR_pos;
    upper_pos = Q3_pos + kOut*IQR_pos;

    pos_inlier_mask = (pos_valid >= lower_pos) & (pos_valid <= upper_pos);
    pos_inliers     = pos_valid(pos_inlier_mask);
else
    pos_inliers = [];
end

% Phía âm
if ~isempty(neg_valid)
    Q1_neg = quantile(neg_valid,0.25);
    Q3_neg = quantile(neg_valid,0.75);
    IQR_neg = Q3_neg - Q1_neg;
    lower_neg = Q1_neg - kOut*IQR_neg;
    upper_neg = Q3_neg + kOut*IQR_neg;

    neg_inlier_mask = (neg_valid >= lower_neg) & (neg_valid <= upper_neg);
    neg_inliers     = neg_valid(neg_inlier_mask);
else
    neg_inliers = [];
end

% Thống kê: mean (thô) và median robust (bỏ ngoại lai)

% Mean “thô”
if ~isempty(pos_valid)
    u_db_pos_mean = mean(pos_valid);
else
    u_db_pos_mean = NaN;
end

if ~isempty(neg_valid)
    u_db_neg_mean = mean(neg_valid);
else
    u_db_neg_mean = NaN;
end

% Median robust
if ~isempty(pos_inliers)
    u_db_pos_med_robust = median(pos_inliers);
else
    u_db_pos_med_robust = NaN;
end

if ~isempty(neg_inliers)
    u_db_neg_med_robust = median(neg_inliers);
else
    u_db_neg_med_robust = NaN;
end

% Deadband đối xứng
u_db_sym_mean   = max(abs([u_db_pos_mean,       u_db_neg_mean]),        [], 'omitnan');
u_db_sym_robust = max(abs([u_db_pos_med_robust, u_db_neg_med_robust]),  [], 'omitnan');

deadbandSummary = struct( ...
    'u_db_pos_each',        u_db_pos_all, ...
    'u_db_neg_each',        u_db_neg_all, ...
    'u_db_pos_mean',        u_db_pos_mean, ...
    'u_db_neg_mean',        u_db_neg_mean, ...
    'u_db_sym_mean',        u_db_sym_mean, ...
    'u_db_pos_med_robust',  u_db_pos_med_robust, ...
    'u_db_neg_med_robust',  u_db_neg_med_robust, ...
    'u_db_sym_robust',      u_db_sym_robust);

deadbandSummary
